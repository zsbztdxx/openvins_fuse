/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2018-2023 Patrick Geneva
 * Copyright (C) 2018-2023 Guoquan Huang
 * Copyright (C) 2018-2023 OpenVINS Contributors
 * Copyright (C) 2018-2019 Kevin Eckenhoff
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <memory>

#include "core/VioManager.h"
#include "core/VioManagerOptions.h"
#include "utils/dataset_reader.h"
#include "state/Propagator.h"
#include "state/State.h"
#include "state/StateHelper.h"
#include "utils/pos_transform.h"

#if ROS_AVAILABLE == 1
#include "ros/ROS1Visualizer.h"
#include "ov_msckf/satnav.h"
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#elif ROS_AVAILABLE == 2
#include "ros/ROS2Visualizer.h"
#include <rclcpp/rclcpp.hpp>
#endif

using namespace ov_msckf;

std::shared_ptr<VioManager> sys;
#if ROS_AVAILABLE == 1
std::shared_ptr<ROS1Visualizer> viz;
ros::Publisher gnss_pub;
std::vector<double> gnss_pub_init_lla;
nav_msgs::Path gnss_path;
#elif ROS_AVAILABLE == 2
std::shared_ptr<ROS2Visualizer> viz;
#endif


std::atomic<bool> thread_update_running;
std::deque<ov_core::CameraData> camera_queue;
std::deque<ov_core::GnssData> gnss_queue;
std::mutex camera_queue_mtx,gnss_queue_mtx;
std::map<int, double> camera_last_timestamp;

void callback_inertial(const sensor_msgs::Imu::ConstPtr &msg) 
{

  // convert into correct format
  ov_core::ImuData message;
  message.timestamp = msg->header.stamp.toSec();
  message.wm << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;
  message.am << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;

  // send it to our VIO system
  sys->feed_measurement_imu(message);
  viz->visualize_odometry(message.timestamp);

  /*{
    if(!sys->initialized() || !sys->initialized_gnss()){
        for(size_t i=0;i<gnss_queue.size();i++){
            sys->feed_measurement_gnss(gnss_queue[i]);
        }
    }
  }*/

  // If the processing queue is currently active / running just return so we can keep getting measurements
  // Otherwise create a second thread to do our update in an async manor
  // The visualization of the state, images, and features will be synchronous with the update!
  if (thread_update_running)
    return;
  thread_update_running = true;
  std::thread thread([&] {
    // Lock on the queue (prevents new images from appending)
    std::lock_guard<std::mutex> lck(camera_queue_mtx);

    // Count how many unique image streams
    std::map<int, bool> unique_cam_ids;
    for (const auto &cam_msg : camera_queue) {
      unique_cam_ids[cam_msg.sensor_ids.at(0)] = true;
    }

    // If we do not have enough unique cameras then we need to wait
    // We should wait till we have one of each camera to ensure we propagate in the correct order
    auto params = sys->get_params();
    size_t num_unique_cameras = (params.state_options.num_cameras == 2) ? 1 : params.state_options.num_cameras;
    if (unique_cam_ids.size() == num_unique_cameras) {

      // Loop through our queue and see if we are able to process any of our camera measurements
      // We are able to process if we have at least one IMU measurement greater than the camera time
      double timestamp_imu_inC = message.timestamp - sys->get_state()->_calib_dt_CAMtoIMU->value()(0);
      while (!camera_queue.empty() && camera_queue.at(0).timestamp < timestamp_imu_inC) {
        auto rT0_1 = boost::posix_time::microsec_clock::local_time();
        double update_dt = 100.0 * (timestamp_imu_inC - camera_queue.at(0).timestamp);
        static int count = 0;
        sys->feed_measurement_camera(camera_queue.at(0));
        viz->visualize();
        
        camera_queue.pop_front();
        auto rT0_2 = boost::posix_time::microsec_clock::local_time();
        double time_total = (rT0_2 - rT0_1).total_microseconds() * 1e-6;
        PRINT_INFO(BLUE "[TIME]: %.4f seconds total (%.1f hz, %.2f ms behind)\n" RESET, time_total, 1.0 / time_total, update_dt);
        PRINT_INFO(BLUE "clones imu size:%d\n" RESET, sys->get_state()->_clones_IMU.size());
        PRINT_INFO(BLUE "count :%d\n" RESET, count++);
        
        if(sys->initialized() && !sys->initialized_gnss()){
            double timestamp = sys->get_state()->_timestamp;
            Eigen::Vector3d pos = sys->get_state()->_imu->pos();
            sys->feed_imu_pos(std::pair<double,Eigen::Vector3d>(timestamp,pos));
            PRINT_INFO(BLUE "!initialized_gnss,feed imu pos\n" RESET);
        }
      }
    }
    thread_update_running = false;
    
  });

  // If we are single threaded, then run single threaded
  // Otherwise detach this thread so it runs in the background!
  if (!sys->get_params().use_multi_threading_subs) {
    thread.join();
  } else {
    thread.detach();
  }
}

void callback_monocular(const sensor_msgs::ImageConstPtr &msg0, int cam_id0)
{

  // Check if we should drop this image
  double timestamp = msg0->header.stamp.toSec();
  double time_delta = 1.0 / sys->get_params().track_frequency;
  if (camera_last_timestamp.find(cam_id0) != camera_last_timestamp.end() && timestamp < camera_last_timestamp.at(cam_id0) + time_delta) {
    return;
  }
  camera_last_timestamp[cam_id0] = timestamp;

  // Get the image
  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvShare(msg0, sensor_msgs::image_encodings::MONO8);
  } catch (cv_bridge::Exception &e) {
    PRINT_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // Create the measurement
  ov_core::CameraData message;
  message.timestamp = cv_ptr->header.stamp.toSec();
  message.sensor_ids.push_back(cam_id0);
  message.images.push_back(cv_ptr->image.clone());

  // Load the mask if we are using it, else it is empty
  // TODO: in the future we should get this from external pixel segmentation
  if (sys->get_params().use_mask) {
    message.masks.push_back(sys->get_params().masks.at(cam_id0));
  } else {
    message.masks.push_back(cv::Mat::zeros(cv_ptr->image.rows, cv_ptr->image.cols, CV_8UC1));
  }

  // append it to our queue of images
  std::lock_guard<std::mutex> lck(camera_queue_mtx);
  camera_queue.push_back(message);
  std::sort(camera_queue.begin(), camera_queue.end());
}

void callback_stereo(const sensor_msgs::ImageConstPtr &msg0, const sensor_msgs::ImageConstPtr &msg1, int cam_id0,
                                     int cam_id1) 
{

  // Check if we should drop this image
  double timestamp = msg0->header.stamp.toSec();
  double time_delta = 1.0 / sys->get_params().track_frequency;
  if (camera_last_timestamp.find(cam_id0) != camera_last_timestamp.end() && timestamp < camera_last_timestamp.at(cam_id0) + time_delta) {
    return;
  }
  camera_last_timestamp[cam_id0] = timestamp;

  // Get the image
  cv_bridge::CvImageConstPtr cv_ptr0;
  try {
    cv_ptr0 = cv_bridge::toCvShare(msg0, sensor_msgs::image_encodings::MONO8);
  } catch (cv_bridge::Exception &e) {
    PRINT_ERROR("cv_bridge exception: %s\n", e.what());
    return;
  }

  // Get the image
  cv_bridge::CvImageConstPtr cv_ptr1;
  try {
    cv_ptr1 = cv_bridge::toCvShare(msg1, sensor_msgs::image_encodings::MONO8);
  } catch (cv_bridge::Exception &e) {
    PRINT_ERROR("cv_bridge exception: %s\n", e.what());
    return;
  }

  // Create the measurement
  ov_core::CameraData message;
  message.timestamp = cv_ptr0->header.stamp.toSec();
  message.sensor_ids.push_back(cam_id0);
  message.sensor_ids.push_back(cam_id1);
  message.images.push_back(cv_ptr0->image.clone());
  message.images.push_back(cv_ptr1->image.clone());

  // Load the mask if we are using it, else it is empty
  // TODO: in the future we should get this from external pixel segmentation
  if (sys->get_params().use_mask) {
    message.masks.push_back(sys->get_params().masks.at(cam_id0));
    message.masks.push_back(sys->get_params().masks.at(cam_id1));
  } else {
    // message.masks.push_back(cv::Mat(cv_ptr0->image.rows, cv_ptr0->image.cols, CV_8UC1, cv::Scalar(255)));
    message.masks.push_back(cv::Mat::zeros(cv_ptr0->image.rows, cv_ptr0->image.cols, CV_8UC1));
    message.masks.push_back(cv::Mat::zeros(cv_ptr1->image.rows, cv_ptr1->image.cols, CV_8UC1));
  }

  // append it to our queue of images
  std::lock_guard<std::mutex> lck(camera_queue_mtx);
  camera_queue.push_back(message);
  std::sort(camera_queue.begin(), camera_queue.end());
}

void callback_gnss(const ov_msckf::satnavConstPtr &msg) 
{
  if(msg->pos_status==0)
  {
    return;
  }
  // convert into correct format
  ov_core::GnssData message;
  message.timestamp = msg->header.header.stamp.toSec();
  message.latitude = msg->latitude;
  message.longitude = msg->longitude;
  message.altitude = msg->altitude;
  message.ve = msg->vel_east;
  message.vn = msg->vel_north;
  message.vu = msg->vel_up;
  message.status_pos = msg->pos_status;
  message.std_latitude = msg->std_latitude;
  message.std_longitude = msg->std_longitude;
  message.std_altitude = msg->std_altitude;
  message.std_ve = msg->std_vel_east;
  message.std_vn = msg->std_vel_north;
  message.std_vu = msg->std_vel_up;

  // append it to our queue of gnss
  std::lock_guard<std::mutex> lck(gnss_queue_mtx);
  gnss_queue.push_back(message);
  std::sort(gnss_queue.begin(), gnss_queue.end());

  if(!sys->initialized_gnss())
  {
    PRINT_ERROR(RED "!sys->initialized_gnss\n" RESET);
    sys->feed_measurement_gnss(message);
  }

  //pub gnss pos 
  if(!sys->initialized_gnss())
  {
    return;
  }
  if(gnss_pub_init_lla.size()==0)
  {
    gnss_pub_init_lla.push_back(sys->get_init_lla()[0]);
    gnss_pub_init_lla.push_back(sys->get_init_lla()[1]);
    gnss_pub_init_lla.push_back(sys->get_init_lla()[2]);
  }

  std::vector<double> lla(3);
  lla[0]=msg->latitude,lla[1]=msg->longitude,lla[2]=msg->altitude;
  std::vector<double> enu = lla2enu(lla,gnss_pub_init_lla);

  gnss_path.header.stamp = msg->header.header.stamp;
  gnss_path.header.frame_id = "global";
  geometry_msgs::PoseStamped curr_path;

  curr_path.header.stamp = msg->header.header.stamp;
  curr_path.header.frame_id = "global";

  Eigen::Vector3d tmp;
  tmp[0] = enu[0],tmp[1] = enu[1],tmp[2] = enu[2];
  tmp = sys->get_R_GNSStoI() * tmp + sys->get_t_GNSStoI();

  curr_path.pose.position.x = tmp[0];
  curr_path.pose.position.y = tmp[1];
  curr_path.pose.position.z = tmp[2];

  curr_path.pose.orientation.x = 0;
  curr_path.pose.orientation.y = 0;
  curr_path.pose.orientation.z = 0;
  curr_path.pose.orientation.w = 1;

  gnss_path.poses.push_back(curr_path);

  gnss_pub.publish(gnss_path);

}

// Main function
int main(int argc, char **argv) {

  // Ensure we have a path, if the user passes it then we should use it
  std::string config_path = "unset_path_to_config.yaml";
  if (argc > 1) {
    config_path = argv[1];
  }

#if ROS_AVAILABLE == 1
  // Launch our ros node
  ros::init(argc, argv, "run_readbag_msckf");
  auto nh = std::make_shared<ros::NodeHandle>("~");
  nh->param<std::string>("config_path", config_path, config_path);
  gnss_pub = nh->advertise<nav_msgs::Path>("gnss_path", 20);
#elif ROS_AVAILABLE == 2
  // Launch our ros node
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.allow_undeclared_parameters(true);
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<rclcpp::Node>("run_readbag_msckf", options);
  node->get_parameter<std::string>("config_path", config_path);
#endif

  // Load the config
  auto parser = std::make_shared<ov_core::YamlParser>(config_path);
#if ROS_AVAILABLE == 1
  parser->set_node_handler(nh);
#elif ROS_AVAILABLE == 2
  parser->set_node(node);
#endif

  // Verbosity
  std::string verbosity = "DEBUG";
  parser->parse_config("verbosity", verbosity);
  ov_core::Printer::setPrintLevel(verbosity);

  // Create our VIO system
  VioManagerOptions params;
  params.print_and_load(parser);
  params.use_multi_threading_subs = true;
  sys = std::make_shared<VioManager>(params);
#if ROS_AVAILABLE == 1
  viz = std::make_shared<ROS1Visualizer>(nh, sys);
#elif ROS_AVAILABLE == 2
  viz = std::make_shared<ROS2Visualizer>(node, sys);
#endif

  // Ensure we read in all parameters required
  if (!parser->successful()) {
    PRINT_ERROR(RED "unable to parse all parameters, please fix\n" RESET);
    std::exit(EXIT_FAILURE);
  }

  // read ros bag and feed
#if ROS_AVAILABLE == 1 || ROS_AVAILABLE == 2
  rosbag::Bag bag;
  std::string bag_path;
  nh->param<std::string>("bag_path", bag_path, bag_path);
  std::string imu_topic,cam0_topic,cam1_topic;
  parser->parse_external("relative_config_imu", "imu0", "rostopic", imu_topic);
  parser->parse_external("relative_config_imucam", "cam0", "rostopic", cam0_topic);
  parser->parse_external("relative_config_imucam", "cam1", "rostopic", cam1_topic);

  bool fuse_gnss = false;
  parser->parse_config("fuse_gnss", fuse_gnss);
  std::string gnss_topic;
  std::vector<double> t_gnss_imu = {0, 0, 0};
  if(fuse_gnss)
  {
    parser->parse_config("gnss_topic", gnss_topic);
    parser->parse_config("t_gnss_imu", t_gnss_imu);
    PRINT_DEBUG("fuse gnss = true, gnss_topic = %s, t_gnss_imu = [%f,%f,%f]\n",
      gnss_topic.c_str(),t_gnss_imu[0],t_gnss_imu[1],t_gnss_imu[2]);
  }

  bag.open(bag_path, rosbag::BagMode::Read);
  if(!bag.isOpen())
  {
    PRINT_ERROR(RED "unable to open bag file\n" RESET);
    std::exit(EXIT_FAILURE);      
  }

  rosbag::View view(bag);

  thread_update_running = false;
  sensor_msgs::ImageConstPtr cam0_msg,cam1_msg;
  bool use_stereo;
  parser->parse_config("use_stereo", use_stereo);
  boost::posix_time::ptime start_time,end_time;
  start_time = boost::posix_time::microsec_clock::local_time();
  for (const auto& msg : view)
  {
    if(!ros::ok()){
      break;
    }
    std::string topic = msg.getTopic();
    if(topic == imu_topic) 
    {
      sensor_msgs::Imu::ConstPtr imu_msg = msg.instantiate<sensor_msgs::Imu>();
      callback_inertial(imu_msg);
    }
    else if(topic == cam1_topic)
    {
      cam1_msg = msg.instantiate<sensor_msgs::Image>();
    }
    else if(topic == cam0_topic) 
    {
      cam0_msg = msg.instantiate<sensor_msgs::Image>();
    }
    else if(topic == gnss_topic)
    {
      ov_msckf::satnavConstPtr gnss_msg = msg.instantiate<ov_msckf::satnav>();
      callback_gnss(gnss_msg);
    }

    if(!use_stereo){
      if(topic == cam0_topic){
        callback_monocular(cam0_msg,0); 
      }
    }else{
      if(topic == cam0_topic || topic == cam1_topic){
        if(cam0_msg && cam1_msg){
          if(cam0_msg->header.stamp.toSec()-cam1_msg->header.stamp.toSec() < 1e-6)
          {
            callback_stereo(cam0_msg,cam1_msg,0,1);
          }
        }
      } 
    }

  }
  end_time = boost::posix_time::microsec_clock::local_time();
  std::cout<<"total time:"<<(end_time - start_time).total_microseconds() * 1e-6 << std::endl;
#endif

  // Final visualization
  viz->visualize_final();
#if ROS_AVAILABLE == 1
  ros::shutdown();
#elif ROS_AVAILABLE == 2
  rclcpp::shutdown();
#endif

  // Done!
  return EXIT_SUCCESS;
}
