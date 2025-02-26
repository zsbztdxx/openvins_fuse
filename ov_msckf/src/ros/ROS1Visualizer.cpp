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

#include "ROS1Visualizer.h"

#include "core/VioManager.h"
#include "ros/ROSVisualizerHelper.h"
#include "sim/Simulator.h"
#include "state/Propagator.h"
#include "state/State.h"
#include "state/StateHelper.h"
#include "utils/dataset_reader.h"
#include "utils/print.h"
#include "utils/sensor_data.h"
#include "utils/pos_transform.h"


using namespace ov_core;
using namespace ov_type;
using namespace ov_msckf;

//save lla path
std::shared_ptr<VioManager> sys;
std::deque<std::pair<double,Eigen::Vector3d>> fuse_pos;
std::mutex fuse_pos_queue_mtx;
bool quit_save_lla = false;
bool save_lla_file = false;

//save imu_integ
std::deque<std::pair<double, Eigen::Vector3d>> imu_integ;
std::mutex imu_integ_queue_mtx;
bool imu_pos_first_initialized = false;
std::pair<double, Eigen::Vector3d> imu_pos_first;
Eigen::Vector3d imu_pos;
std::deque<std::pair<double, Eigen::Vector3d>> imu_pos_queue;
bool quit_save_imu_integ = false;
bool save_imu_integ = false;

void save_imu_integ_thread(const std::string path) {
  FILE* fp = fopen(path.c_str(),"w");
  bool save_as_rtkplot_pos = false;
  if(path.find(".pos") != std::string::npos){
    save_as_rtkplot_pos = true;
  }
  if (save_as_rtkplot_pos) {
    fprintf(fp, "%% UTC                     latitude(deg)  longitude(deg)  height(m) Q  ns   sdn(m)   sde(m)   sdu(m)  sdne(m)  sdeu(m)  sdun(m) age(s)  ratio\n");
  } else {
    fprintf(fp,"#timestamp,latitude,longitude,altitude\n");
  }

  fflush(fp);
  while(!quit_save_imu_integ) {
    if(imu_integ.size() > 0 && sys->initialized_gnss())
    {
      std::lock_guard<std::mutex> lck(imu_integ_queue_mtx);
      while (!imu_integ.empty())
      {
        double timestamp = imu_integ.front().first;
        Eigen::Vector3d pos_imu = imu_integ.front().second;
        Eigen::Vector3d pos_gnss = sys->get_R_GNSStoI().transpose()*(pos_imu-sys->get_t_GNSStoI());
        std::vector<double> tmp(3);
        tmp[0] = pos_gnss[0],tmp[1] = pos_gnss[1],tmp[2] = pos_gnss[2];
        std::vector<double> lla = enu2lla(tmp,sys->get_init_lla());

        if(save_as_rtkplot_pos){
          ros::WallTime time = ros::WallTime(timestamp + 8*3600);
          int year = time.toBoost().date().year();
          int month = time.toBoost().date().month();
          int day = time.toBoost().date().day();
          int hours = time.toBoost().time_of_day().hours();
          int minutes = time.toBoost().time_of_day().minutes();
          int seconds = time.toBoost().time_of_day().seconds();
          int milliseconds = round(time.toBoost().time_of_day().fractional_seconds()/1000.0);
          
          char timestr[64];

          sprintf(timestr,"%04d/%02d/%02d %02d:%02d:%02d.%03d",year,month,day,hours,minutes,seconds,milliseconds);
          fprintf(fp, "%s   %.09f   %.9f   %.4f   4   0   0   0   0   0   0   0   0   0\n", timestr, lla[0], lla[1], lla[2]);
        }else{
          fprintf(fp, "%.6f,%.9f,%.9f,%.4f\n", timestamp, lla[0], lla[1], lla[2]);
        }
        imu_integ.pop_front();
      }
      fflush(fp);
    }
    usleep(100*1000);
  }
  fclose(fp);
}

void save_lla_thread(const std::string path) {

    FILE* fp = fopen(path.c_str(),"w");
    bool save_as_rtkplot_pos = false;
    if(path.find(".pos") != std::string::npos){
      save_as_rtkplot_pos = true;
    }

    if(save_as_rtkplot_pos){
      fprintf(fp, "%% UTC                    latitude(deg)  longitude(deg)  height(m) Q  ns   sdn(m)   sde(m)   sdu(m)  sdne(m)  sdeu(m)  sdun(m) age(s)  ratio\n");
    }else{
      fprintf(fp,"#timestamp,latitude,longitude,altitude\n");
    }
    
    fflush(fp);
    while(!quit_save_lla)
    {
        if(fuse_pos.size()>0 && sys && sys->initialized_gnss())
        {
          std::lock_guard<std::mutex> lck(fuse_pos_queue_mtx);
          while (!fuse_pos.empty())
          {
            double timestamp = fuse_pos.front().first;
            Eigen::Vector3d pos_imu = fuse_pos.front().second;
            Eigen::Vector3d pos_gnss = sys->get_R_GNSStoI().transpose()*(pos_imu-sys->get_t_GNSStoI());
            std::vector<double> tmp(3);
            tmp[0] = pos_gnss[0],tmp[1] = pos_gnss[1],tmp[2] = pos_gnss[2];
            std::vector<double> lla = enu2lla(tmp,sys->get_init_lla());

            if(save_as_rtkplot_pos){
              // ros::WallTime time = ros::WallTime(timestamp + 8*3600);
              ros::WallTime time = ros::WallTime(timestamp);
              int year = time.toBoost().date().year();
              int month = time.toBoost().date().month();
              int day = time.toBoost().date().day();
              int hours = time.toBoost().time_of_day().hours();
              int minutes = time.toBoost().time_of_day().minutes();
              int seconds = time.toBoost().time_of_day().seconds();
              int milliseconds = round(time.toBoost().time_of_day().fractional_seconds()/1000.0);
              
              char timestr[64];

              sprintf(timestr,"%04d/%02d/%02d %02d:%02d:%02d.%03d",year,month,day,hours,minutes,seconds,milliseconds);
              fprintf(fp, "%s   %.09f   %.9f   %.4f   4   0   0   0   0   0   0   0   0   0\n", timestr, lla[0], lla[1], lla[2]);
            }else{
              fprintf(fp, "%.6f,%.9f,%.9f,%.4f\n", timestamp, lla[0], lla[1], lla[2]);
            }
            
            fuse_pos.pop_front();
          }
          fflush(fp);
        }
        usleep(100*1000);
    }
    fclose(fp);
}

struct gtime_t {
    time_t time;
    double sec;
};

inline gtime_t epoch2time(const double *ep) {
    const int doy[]={1,32,60,91,121,152,182,213,244,274,305,335};

    gtime_t time={0};

    int days,sec,year=(int)ep[0],mon=(int)ep[1],day=(int)ep[2];
    
    if (year<1970||2099<year||mon<1||12<mon) return time;
    
    /* leap year if year%4==0 in 1901-2099 */
    days=(year-1970)*365+(year-1969)/4+doy[mon-1]+day-2+(year%4==0&&mon>=3?1:0);
    sec=(int)floor(ep[5]);
    time.time=(time_t)days*86400+(int)ep[3]*3600+(int)ep[4]*60+sec;
    time.sec=ep[5]-sec;
    return time;
}

inline gtime_t gpst2time(int week, double sec) {
    const double gpst0[6] = {1980, 1, 6, 0, 0, 0}; /* gps time reference */

    gtime_t t = epoch2time(gpst0);

    // std::cerr << "epoch2time为:" << t.time << "epoch2time为" << t.sec << std::endl;

    if (sec < -1E9 || 1E9 < sec)
    {
        t.time += (time_t)(86400 * 7 * week);
        t.sec = 0;
    }
    else
    {
      t.time += (time_t)(86400 * 7 * week) + (int)sec;
      t.sec = sec - (int)sec;
    }
    return t;
}

inline void judge_ggp_rover_fix_level(int level) {
  switch(level) {
    case 0: 
      PRINT_INFO(RED "GGP分级协议固定解等级：非固定解\n" RESET);
      break;
    case 1:
      PRINT_INFO(GREEN "GGP分级协议固定解等级：最优固定解\n" RESET);
      break;
    case 2:
      PRINT_INFO(GREEN "GGP分级协议固定解等级：次优固定解\n" RESET);
      break;
    case 3:
      PRINT_INFO(GREEN "GGP分级协议固定解等级：不可靠固定解\n" RESET);
      break;
    default:
      PRINT_INFO(RED "非GGP分级协议等级状态！\n" RESET);
      break;
  }
}

ROS1Visualizer::ROS1Visualizer(std::shared_ptr<ros::NodeHandle> nh, std::shared_ptr<VioManager> app, std::shared_ptr<Simulator> sim)
    : _nh(nh), _app(app), _sim(sim), thread_update_running(false) {

  sys = app;

  loselock_period = _app->get_params().loselock_period;

  // Setup our transform broadcaster
  mTfBr = std::make_shared<tf::TransformBroadcaster>();

  // Create image transport
  image_transport::ImageTransport it(*_nh);

  // Setup pose and path publisher
  // pub_poseimu = nh->advertise<geometry_msgs::PoseWithCovarianceStamped>("poseimu", 2);
  pub_poseimu = nh->advertise<geometry_msgs::PoseStamped>("poseimu", 2);
  PRINT_DEBUG("Publishing: %s\n", pub_poseimu.getTopic().c_str());
  pub_odomimu = nh->advertise<nav_msgs::Odometry>("odomimu", 2);
  PRINT_DEBUG("Publishing: %s\n", pub_odomimu.getTopic().c_str());
  pub_pathimu = nh->advertise<nav_msgs::Path>("pathimu", 2);
  PRINT_DEBUG("Publishing: %s\n", pub_pathimu.getTopic().c_str());

  // 3D points publishing
  pub_points_msckf = nh->advertise<sensor_msgs::PointCloud2>("points_msckf", 2);
  PRINT_DEBUG("Publishing: %s\n", pub_points_msckf.getTopic().c_str());
  pub_points_slam = nh->advertise<sensor_msgs::PointCloud2>("points_slam", 2);
  PRINT_DEBUG("Publishing: %s\n", pub_points_msckf.getTopic().c_str());
  pub_points_aruco = nh->advertise<sensor_msgs::PointCloud2>("points_aruco", 2);
  PRINT_DEBUG("Publishing: %s\n", pub_points_aruco.getTopic().c_str());
  pub_points_sim = nh->advertise<sensor_msgs::PointCloud2>("points_sim", 2);
  PRINT_DEBUG("Publishing: %s\n", pub_points_sim.getTopic().c_str());

  // Our tracking image
  it_pub_tracks = it.advertise("trackhist", 2);
  PRINT_DEBUG("Publishing: %s\n", it_pub_tracks.getTopic().c_str());

  // Groundtruth publishers
  pub_posegt = nh->advertise<geometry_msgs::PoseStamped>("posegt", 2);
  PRINT_DEBUG("Publishing: %s\n", pub_posegt.getTopic().c_str());
  pub_pathgt = nh->advertise<nav_msgs::Path>("pathgt", 2);
  PRINT_DEBUG("Publishing: %s\n", pub_pathgt.getTopic().c_str());

  //satnav path publishers
  pub_satnav = nh->advertise<nav_msgs::Path>("satnav_path", 2);
  PRINT_DEBUG("Publishing: %s\n", pub_satnav.getTopic().c_str());

  //inspva path publishers
  pub_inspva = nh->advertise<nav_msgs::Path>("inspva_path", 2);
  PRINT_DEBUG("Publishing: %s\n", pub_inspva.getTopic().c_str());

  // Loop closure publishers
  pub_loop_pose = nh->advertise<nav_msgs::Odometry>("loop_pose", 2);
  pub_loop_point = nh->advertise<sensor_msgs::PointCloud>("loop_feats", 2);
  pub_loop_extrinsic = nh->advertise<nav_msgs::Odometry>("loop_extrinsic", 2);
  pub_loop_intrinsics = nh->advertise<sensor_msgs::CameraInfo>("loop_intrinsics", 2);
  it_pub_loop_img_depth = it.advertise("loop_depth", 2);
  it_pub_loop_img_depth_color = it.advertise("loop_depth_colored", 2);

  // option to enable publishing of global to IMU transformation
  nh->param<bool>("publish_global_to_imu_tf", publish_global2imu_tf, true);
  nh->param<bool>("publish_calibration_tf", publish_calibration_tf, true);

  // Load groundtruth if we have it and are not doing simulation
  // NOTE: needs to be a csv ASL format file
  if (nh->hasParam("path_gt") && _sim == nullptr) {
    std::string path_to_gt;
    nh->param<std::string>("path_gt", path_to_gt, "");
    if (!path_to_gt.empty()) {
      DatasetReader::load_gt_file(path_to_gt, gt_states);
      PRINT_DEBUG("gt file path is: %s\n", path_to_gt.c_str());
    }
  }

  // Load if we should save the total state to file
  // If so, then open the file and create folders as needed
  nh->param<bool>("save_total_state", save_total_state, false);
  if (save_total_state) {

    // files we will open
    std::string filepath_est, filepath_std, filepath_gt;
    nh->param<std::string>("filepath_est", filepath_est, "state_estimate.txt");
    nh->param<std::string>("filepath_std", filepath_std, "state_deviation.txt");
    nh->param<std::string>("filepath_gt", filepath_gt, "state_groundtruth.txt");

    // If it exists, then delete it
    if (boost::filesystem::exists(filepath_est))
      boost::filesystem::remove(filepath_est);
    if (boost::filesystem::exists(filepath_std))
      boost::filesystem::remove(filepath_std);

    // Create folder path to this location if not exists
    boost::filesystem::create_directories(boost::filesystem::path(filepath_est.c_str()).parent_path());
    boost::filesystem::create_directories(boost::filesystem::path(filepath_std.c_str()).parent_path());

    // Open the files
    of_state_est.open(filepath_est.c_str());
    of_state_std.open(filepath_std.c_str());
    of_state_est << "# timestamp(s) q p v bg ba cam_imu_dt num_cam cam0_k cam0_d cam0_rot cam0_trans ... imu_model dw da tg wtoI atoI etc"
                 << std::endl;
    of_state_std << "# timestamp(s) q p v bg ba cam_imu_dt num_cam cam0_k cam0_d cam0_rot cam0_trans ... imu_model dw da tg wtoI atoI etc"
                 << std::endl;

    // Groundtruth if we are simulating
    if (_sim != nullptr) {
      if (boost::filesystem::exists(filepath_gt))
        boost::filesystem::remove(filepath_gt);
      boost::filesystem::create_directories(boost::filesystem::path(filepath_gt.c_str()).parent_path());
      of_state_gt.open(filepath_gt.c_str());
      of_state_gt << "# timestamp(s) q p v bg ba cam_imu_dt num_cam cam0_k cam0_d cam0_rot cam0_trans ... imu_model dw da tg wtoI atoI etc"
                  << std::endl;
    }
  }

  // Start thread for the image publishing
  if (_app->get_params().use_multi_threading_pubs) {
    std::thread thread([&] {
      ros::Rate loop_rate(20);
      while (ros::ok()) {
        publish_images();
        loop_rate.sleep();
      }
    });
    thread.detach();
  }
}

void ROS1Visualizer::setup_subscribers(std::shared_ptr<ov_core::YamlParser> parser) {

  // We need a valid parser
  assert(parser != nullptr);

  // Create imu subscriber (handle legacy ros param info)
  std::string topic_imu;
  _nh->param<std::string>("topic_imu", topic_imu, "/imu0");
  parser->parse_external("relative_config_imu", "imu0", "rostopic", topic_imu);
  sub_imu = _nh->subscribe(topic_imu, 1000, &ROS1Visualizer::callback_inertial, this);
  PRINT_INFO("subscribing to IMU: %s\n", topic_imu.c_str());

  // Create gnss subscriber
  bool fuse_gnss = false;
  parser->parse_config("fuse_gnss", fuse_gnss ,false);
  std::string topic_satnav;
  std::string topic_ggp;
  std::string topic_inspva;
  if(fuse_gnss) {
    parser->parse_config("satnav_topic", topic_satnav);
    sub_satnav = _nh->subscribe(topic_satnav, 100, &ROS1Visualizer::callback_satnav, this);
    // sub_gnss = _nh->subscribe(gnss_topic, 1000, &ROS1Visualizer::callback_insnav, this);
    PRINT_INFO("fuse gnss = true, satnav_topic = %s\n",topic_satnav.c_str());

    parser->parse_config("ggp_topic", topic_ggp);
    sub_ggp = _nh->subscribe(topic_ggp, 100, &ROS1Visualizer::callback_ggp, this);
    PRINT_INFO("subscribing to ggp: %s\n", topic_ggp.c_str());
    
    parser->parse_config("inspva_topic", topic_inspva);
    sub_inspva = _nh->subscribe(topic_inspva, 1000, &ROS1Visualizer::callback_inspva, this);
    PRINT_INFO("subscribing to inspva: %s\n", topic_inspva.c_str());
  }

  // Logic for sync stereo subscriber
  // https://answers.ros.org/question/96346/subscribe-to-two-image_raws-with-one-function/?answer=96491#post-id-96491
  if (_app->get_params().state_options.num_cameras == 2) {
    // Read in the topics
    std::string cam_topic0, cam_topic1;
    _nh->param<std::string>("topic_camera" + std::to_string(0), cam_topic0, "/cam" + std::to_string(0) + "/image_raw");
    _nh->param<std::string>("topic_camera" + std::to_string(1), cam_topic1, "/cam" + std::to_string(1) + "/image_raw");
    parser->parse_external("relative_config_imucam", "cam" + std::to_string(0), "rostopic", cam_topic0);
    parser->parse_external("relative_config_imucam", "cam" + std::to_string(1), "rostopic", cam_topic1);
    // Create sync filter (they have unique pointers internally, so we have to use move logic here...)
    auto image_sub0 = std::make_shared<message_filters::Subscriber<sensor_msgs::Image>>(*_nh, cam_topic0, 1);
    auto image_sub1 = std::make_shared<message_filters::Subscriber<sensor_msgs::Image>>(*_nh, cam_topic1, 1);
    auto sync = std::make_shared<message_filters::Synchronizer<sync_pol>>(sync_pol(10), *image_sub0, *image_sub1);
    sync->registerCallback(boost::bind(&ROS1Visualizer::callback_stereo, this, _1, _2, 0, 1));
    // Append to our vector of subscribers
    sync_cam.push_back(sync);
    sync_subs_cam.push_back(image_sub0);
    sync_subs_cam.push_back(image_sub1);
    PRINT_INFO("subscribing to cam (stereo): %s\n", cam_topic0.c_str());
    PRINT_INFO("subscribing to cam (stereo): %s\n", cam_topic1.c_str());
  } else {
    // Now we should add any non-stereo callbacks here
    for (int i = 0; i < _app->get_params().state_options.num_cameras; i++) {
      // read in the topic
      std::string cam_topic;
      _nh->param<std::string>("topic_camera" + std::to_string(i), cam_topic, "/cam" + std::to_string(i) + "/image_raw");
      parser->parse_external("relative_config_imucam", "cam" + std::to_string(i), "rostopic", cam_topic);
      // create subscriber
      subs_cam.push_back(_nh->subscribe<sensor_msgs::Image>(cam_topic, 10, boost::bind(&ROS1Visualizer::callback_monocular, this, _1, i)));
      PRINT_INFO("subscribing to cam (mono): %s\n", cam_topic.c_str());
    }
  }

  //save lla file
  parser->parse_config("fuse_gnss", save_lla_file ,false);
  if(save_lla_file) {
    std::string lla_filepath = "/tmp/ov_estimate_lla.txt";
    parser->parse_config("lla_filepath", lla_filepath);
    std::thread thread(&save_lla_thread,lla_filepath);
    thread.detach();
  }

  //save imu_integ file
  parser->parse_config("save_imu_integ", save_imu_integ, false);
  if (save_imu_integ) {
    std::string imu_integ_filepath = "/tmp/imu_integ.pos";
    parser->parse_config("imu_integ_filepath", imu_integ_filepath);
    std::thread thread(&save_imu_integ_thread, imu_integ_filepath);
    thread.detach();
  }
}

void ROS1Visualizer::visualize() {

  // Return if we have already visualized
  if (last_visualization_timestamp == _app->get_state()->_timestamp && _app->initialized())
    return;
  last_visualization_timestamp = _app->get_state()->_timestamp;

  // Start timing
  // boost::posix_time::ptime rT0_1, rT0_2;
  // rT0_1 = boost::posix_time::microsec_clock::local_time();

  // publish current image (only if not multi-threaded)
  if (!_app->get_params().use_multi_threading_pubs)
    publish_images();

  // Return if we have not inited
  if (!_app->initialized())
    return;

  // Save the start time of this dataset
  if (!start_time_set) {
    rT1 = boost::posix_time::microsec_clock::local_time();
    start_time_set = true;
  }

  // publish state
  publish_state();

  // publish points
  publish_features();

  // Publish gt if we have it
  publish_groundtruth();

  // Publish keyframe information
  publish_loopclosure_information();

  // Save total state
  if (save_total_state) {
    ROSVisualizerHelper::sim_save_total_state_to_file(_app->get_state(), _sim, of_state_est, of_state_std, of_state_gt);
  }

  // Print how much time it took to publish / displaying things
  // rT0_2 = boost::posix_time::microsec_clock::local_time();
  // double time_total = (rT0_2 - rT0_1).total_microseconds() * 1e-6;
  // PRINT_DEBUG(BLUE "[TIME]: %.4f seconds for visualization\n" RESET, time_total);
}

void ROS1Visualizer::visualize_odometry(double timestamp, std::deque<pair<double, Eigen::Vector3d>> &imu_pos_queue) {

  // Return if we have not inited
  if (!_app->initialized())
    return;

  // Get fast propagate state at the desired timestamp
  std::shared_ptr<State> state = _app->get_state();
  Eigen::Matrix<double, 13, 1> state_plus = Eigen::Matrix<double, 13, 1>::Zero();
  Eigen::Matrix<double, 12, 12> cov_plus = Eigen::Matrix<double, 12, 12>::Zero();
  if (!_app->get_propagator()->fast_state_propagate(state, timestamp, state_plus, cov_plus))
    return;

  //  // Get the simulated groundtruth so we can evaulate the error in respect to it
  //  // NOTE: we get the true time in the IMU clock frame
  //  if (_sim != nullptr) {
  //    Eigen::Matrix<double, 17, 1> state_gt;
  //    if (_sim->get_state(timestamp, state_gt)) {
  //      // Difference between positions
  //      double dx = state_plus(4, 0) - state_gt(5, 0);
  //      double dy = state_plus(5, 0) - state_gt(6, 0);
  //      double dz = state_plus(6, 0) - state_gt(7, 0);
  //      double err_pos = std::sqrt(dx * dx + dy * dy + dz * dz);
  //      // Quaternion error
  //      Eigen::Matrix<double, 4, 1> quat_gt, quat_st, quat_diff;
  //      quat_gt << state_gt(1, 0), state_gt(2, 0), state_gt(3, 0), state_gt(4, 0);
  //      quat_st << state_plus(0, 0), state_plus(1, 0), state_plus(2, 0), state_plus(3, 0);
  //      quat_diff = quat_multiply(quat_st, Inv(quat_gt));
  //      double err_ori = (180 / M_PI) * 2 * quat_diff.block(0, 0, 3, 1).norm();
  //      // Calculate NEES values
  //      Eigen::Vector3d quat_diff_vec = quat_diff.block(0, 0, 3, 1);
  //      Eigen::Vector3d cov_vec = cov_plus.block(0, 0, 3, 3).inverse() * 2 * quat_diff.block(0, 0, 3, 1);
  //      double ori_nees = 2 * quat_diff_vec.dot(cov_vec);
  //      Eigen::Vector3d errpos = state_plus.block(4, 0, 3, 1) - state_gt.block(5, 0, 3, 1);
  //      double pos_nees = errpos.transpose() * cov_plus.block(3, 3, 3, 3).inverse() * errpos;
  //      PRINT_INFO(REDPURPLE "error to gt => %.3f, %.3f (deg,m) | nees => %.1f, %.1f (ori,pos) \n" RESET, err_ori, err_pos, ori_nees,
  //                 pos_nees);
  //    }
  //  }

  // Publish our odometry message if requested
  // if (pub_odomimu.getNumSubscribers() != 0) {

    nav_msgs::Odometry odomIinM;
    odomIinM.header.stamp = ros::Time(timestamp);
    odomIinM.header.frame_id = "global";

    // The POSE component (orientation and position)
    odomIinM.pose.pose.orientation.x = state_plus(0);
    odomIinM.pose.pose.orientation.y = state_plus(1);
    odomIinM.pose.pose.orientation.z = state_plus(2);
    odomIinM.pose.pose.orientation.w = state_plus(3);
    odomIinM.pose.pose.position.x = state_plus(4);
    odomIinM.pose.pose.position.y = state_plus(5);
    odomIinM.pose.pose.position.z = state_plus(6);
    imu_pos << state_plus(4), state_plus(5), state_plus(6);
    imu_pos_queue.push_back(std::make_pair(timestamp, imu_pos));
    // PRINT_INFO(GREEN "执行visualize_odom时，imu_pos的值为：x：%.4f y：%.4f z：%.4f\n" RESET, imu_pos(0), imu_pos(1), imu_pos(2));


    // The TWIST component (angular and linear velocities)
    odomIinM.child_frame_id = "imu";
    odomIinM.twist.twist.linear.x = state_plus(7);   // vel in local frame
    odomIinM.twist.twist.linear.y = state_plus(8);   // vel in local frame
    odomIinM.twist.twist.linear.z = state_plus(9);   // vel in local frame
    odomIinM.twist.twist.angular.x = state_plus(10); // we do not estimate this...
    odomIinM.twist.twist.angular.y = state_plus(11); // we do not estimate this...
    odomIinM.twist.twist.angular.z = state_plus(12); // we do not estimate this...

    // Finally set the covariance in the message (in the order position then orientation as per ros convention)
    Eigen::Matrix<double, 12, 12> Phi = Eigen::Matrix<double, 12, 12>::Zero();
    Phi.block(0, 3, 3, 3).setIdentity();
    Phi.block(3, 0, 3, 3).setIdentity();
    Phi.block(6, 6, 6, 6).setIdentity();
    cov_plus = Phi * cov_plus * Phi.transpose();
    for (int r = 0; r < 6; r++) {
      for (int c = 0; c < 6; c++) {
        odomIinM.pose.covariance[6 * r + c] = cov_plus(r, c);
      }
    }
    for (int r = 0; r < 6; r++) {
      for (int c = 0; c < 6; c++) {
        odomIinM.twist.covariance[6 * r + c] = cov_plus(r + 6, c + 6);
      }
    }
    pub_odomimu.publish(odomIinM);
  // }

  // Publish our transform on TF
  // NOTE: since we use JPL we have an implicit conversion to Hamilton when we publish
  // NOTE: a rotation from GtoI in JPL has the same xyzw as a ItoG Hamilton rotation
  auto odom_pose = std::make_shared<ov_type::PoseJPL>();
  odom_pose->set_value(state_plus.block(0, 0, 7, 1));
  tf::StampedTransform trans = ROSVisualizerHelper::get_stamped_transform_from_pose(odom_pose, false);
  trans.frame_id_ = "global";
  trans.child_frame_id_ = "imu";
  if (publish_global2imu_tf) {
    mTfBr->sendTransform(trans);
  }

  // Loop through each camera calibration and publish it
  for (const auto &calib : state->_calib_IMUtoCAM) {
    tf::StampedTransform trans_calib = ROSVisualizerHelper::get_stamped_transform_from_pose(calib.second, true);
    trans_calib.frame_id_ = "imu";
    trans_calib.child_frame_id_ = "cam" + std::to_string(calib.first);
    if (publish_calibration_tf) {
      mTfBr->sendTransform(trans_calib);
    }
  }
}

void ROS1Visualizer::visualize_final() {

  // Final time offset value
  if (_app->get_state()->_options.do_calib_camera_timeoffset) {
    PRINT_INFO(REDPURPLE "camera-imu timeoffset = %.5f\n\n" RESET, _app->get_state()->_calib_dt_CAMtoIMU->value()(0));
  }

  // Final camera intrinsics
  if (_app->get_state()->_options.do_calib_camera_intrinsics) {
    for (int i = 0; i < _app->get_state()->_options.num_cameras; i++) {
      std::shared_ptr<Vec> calib = _app->get_state()->_cam_intrinsics.at(i);
      PRINT_INFO(REDPURPLE "cam%d intrinsics:\n" RESET, (int)i);
      PRINT_INFO(REDPURPLE "%.3f,%.3f,%.3f,%.3f\n" RESET, calib->value()(0), calib->value()(1), calib->value()(2), calib->value()(3));
      PRINT_INFO(REDPURPLE "%.5f,%.5f,%.5f,%.5f\n\n" RESET, calib->value()(4), calib->value()(5), calib->value()(6), calib->value()(7));
    }
  }

  // Final camera extrinsics
  if (_app->get_state()->_options.do_calib_camera_pose) {
    for (int i = 0; i < _app->get_state()->_options.num_cameras; i++) {
      std::shared_ptr<PoseJPL> calib = _app->get_state()->_calib_IMUtoCAM.at(i);
      Eigen::Matrix4d T_CtoI = Eigen::Matrix4d::Identity();
      T_CtoI.block(0, 0, 3, 3) = quat_2_Rot(calib->quat()).transpose();
      T_CtoI.block(0, 3, 3, 1) = -T_CtoI.block(0, 0, 3, 3) * calib->pos();
      PRINT_INFO(REDPURPLE "T_C%dtoI:\n" RESET, i);
      PRINT_INFO(REDPURPLE "%.3f,%.3f,%.3f,%.3f,\n" RESET, T_CtoI(0, 0), T_CtoI(0, 1), T_CtoI(0, 2), T_CtoI(0, 3));
      PRINT_INFO(REDPURPLE "%.3f,%.3f,%.3f,%.3f,\n" RESET, T_CtoI(1, 0), T_CtoI(1, 1), T_CtoI(1, 2), T_CtoI(1, 3));
      PRINT_INFO(REDPURPLE "%.3f,%.3f,%.3f,%.3f,\n" RESET, T_CtoI(2, 0), T_CtoI(2, 1), T_CtoI(2, 2), T_CtoI(2, 3));
      PRINT_INFO(REDPURPLE "%.3f,%.3f,%.3f,%.3f\n\n" RESET, T_CtoI(3, 0), T_CtoI(3, 1), T_CtoI(3, 2), T_CtoI(3, 3));
    }
  }

  // IMU intrinsics
  if (_app->get_state()->_options.do_calib_imu_intrinsics) {
    Eigen::Matrix3d Dw = State::Dm(_app->get_state()->_options.imu_model, _app->get_state()->_calib_imu_dw->value());
    Eigen::Matrix3d Da = State::Dm(_app->get_state()->_options.imu_model, _app->get_state()->_calib_imu_da->value());
    Eigen::Matrix3d Tw = Dw.colPivHouseholderQr().solve(Eigen::Matrix3d::Identity());
    Eigen::Matrix3d Ta = Da.colPivHouseholderQr().solve(Eigen::Matrix3d::Identity());
    Eigen::Matrix3d R_IMUtoACC = _app->get_state()->_calib_imu_ACCtoIMU->Rot().transpose();
    Eigen::Matrix3d R_IMUtoGYRO = _app->get_state()->_calib_imu_GYROtoIMU->Rot().transpose();
    PRINT_INFO(REDPURPLE "Tw:\n" RESET);
    PRINT_INFO(REDPURPLE "%.5f,%.5f,%.5f,\n" RESET, Tw(0, 0), Tw(0, 1), Tw(0, 2));
    PRINT_INFO(REDPURPLE "%.5f,%.5f,%.5f,\n" RESET, Tw(1, 0), Tw(1, 1), Tw(1, 2));
    PRINT_INFO(REDPURPLE "%.5f,%.5f,%.5f\n\n" RESET, Tw(2, 0), Tw(2, 1), Tw(2, 2));
    PRINT_INFO(REDPURPLE "Ta:\n" RESET);
    PRINT_INFO(REDPURPLE "%.5f,%.5f,%.5f,\n" RESET, Ta(0, 0), Ta(0, 1), Ta(0, 2));
    PRINT_INFO(REDPURPLE "%.5f,%.5f,%.5f,\n" RESET, Ta(1, 0), Ta(1, 1), Ta(1, 2));
    PRINT_INFO(REDPURPLE "%.5f,%.5f,%.5f\n\n" RESET, Ta(2, 0), Ta(2, 1), Ta(2, 2));
    PRINT_INFO(REDPURPLE "R_IMUtoACC:\n" RESET);
    PRINT_INFO(REDPURPLE "%.5f,%.5f,%.5f,\n" RESET, R_IMUtoACC(0, 0), R_IMUtoACC(0, 1), R_IMUtoACC(0, 2));
    PRINT_INFO(REDPURPLE "%.5f,%.5f,%.5f,\n" RESET, R_IMUtoACC(1, 0), R_IMUtoACC(1, 1), R_IMUtoACC(1, 2));
    PRINT_INFO(REDPURPLE "%.5f,%.5f,%.5f\n\n" RESET, R_IMUtoACC(2, 0), R_IMUtoACC(2, 1), R_IMUtoACC(2, 2));
    PRINT_INFO(REDPURPLE "R_IMUtoGYRO:\n" RESET);
    PRINT_INFO(REDPURPLE "%.5f,%.5f,%.5f,\n" RESET, R_IMUtoGYRO(0, 0), R_IMUtoGYRO(0, 1), R_IMUtoGYRO(0, 2));
    PRINT_INFO(REDPURPLE "%.5f,%.5f,%.5f,\n" RESET, R_IMUtoGYRO(1, 0), R_IMUtoGYRO(1, 1), R_IMUtoGYRO(1, 2));
    PRINT_INFO(REDPURPLE "%.5f,%.5f,%.5f\n\n" RESET, R_IMUtoGYRO(2, 0), R_IMUtoGYRO(2, 1), R_IMUtoGYRO(2, 2));
  }

  // IMU intrinsics gravity sensitivity
  if (_app->get_state()->_options.do_calib_imu_g_sensitivity) {
    Eigen::Matrix3d Tg = State::Tg(_app->get_state()->_calib_imu_tg->value());
    PRINT_INFO(REDPURPLE "Tg:\n" RESET);
    PRINT_INFO(REDPURPLE "%.6f,%.6f,%.6f,\n" RESET, Tg(0, 0), Tg(0, 1), Tg(0, 2));
    PRINT_INFO(REDPURPLE "%.6f,%.6f,%.6f,\n" RESET, Tg(1, 0), Tg(1, 1), Tg(1, 2));
    PRINT_INFO(REDPURPLE "%.6f,%.6f,%.6f\n\n" RESET, Tg(2, 0), Tg(2, 1), Tg(2, 2));
  }

  // Publish RMSE if we have it
  if (!gt_states.empty()) {
    PRINT_INFO(REDPURPLE "RMSE: %.3f (deg) orientation\n" RESET, std::sqrt(summed_mse_ori / summed_number));
    PRINT_INFO(REDPURPLE "RMSE: %.3f (m) position\n\n" RESET, std::sqrt(summed_mse_pos / summed_number));
  }

  // Publish RMSE and NEES if doing simulation
  if (_sim != nullptr) {
    PRINT_INFO(REDPURPLE "RMSE: %.3f (deg) orientation\n" RESET, std::sqrt(summed_mse_ori / summed_number));
    PRINT_INFO(REDPURPLE "RMSE: %.3f (m) position\n\n" RESET, std::sqrt(summed_mse_pos / summed_number));
    PRINT_INFO(REDPURPLE "NEES: %.3f (deg) orientation\n" RESET, summed_nees_ori / summed_number);
    PRINT_INFO(REDPURPLE "NEES: %.3f (m) position\n\n" RESET, summed_nees_pos / summed_number);
  }

  // Print the total time
  rT2 = boost::posix_time::microsec_clock::local_time();
  PRINT_INFO(REDPURPLE "TIME: %.3f seconds\n\n" RESET, (rT2 - rT1).total_microseconds() * 1e-6);

  quit_save_lla = true;
  quit_save_imu_integ = true;
}

void ROS1Visualizer::callback_inertial(const sensor_msgs::Imu::ConstPtr &msg) {

  // convert into correct format
  ov_core::ImuData message;
  message.timestamp = msg->header.stamp.toSec();
  message.wm << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;
  message.am << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;

  // extract Y/M/D of imu time to fill the ggp timestamp
  unix_ref_imutime = message.timestamp;
  if(unix_ref_imutime == 0) {
    unix_ref_imutime_got = false;
    return;
  } else {
    unix_ref_imutime_got = true;
  }

  // send it to our VIO system
  _app->feed_measurement_imu(message);
  visualize_odometry(message.timestamp, imu_pos_queue);

  // extract imu integration result and publish
  if (save_imu_integ && _app->initialized() && imu_pos_queue.size() > 2) {
    std::lock_guard<std::mutex> lck(imu_integ_queue_mtx);
    if (!imu_pos_first_initialized) {
      imu_pos_first = imu_pos_queue.front();
      imu_pos_first_initialized = true;
    }
    auto it0 = imu_pos_queue.begin();
    while (it0 != imu_pos_queue.end()) {
      if (it0->first - imu_pos_first.first >= 0.1) {
        imu_integ.emplace_back(*it0);
        imu_pos_first = *it0;
        // PRINT_INFO(GREEN "存入imu_integ队列的第一个值为：time：%.4f x：%.4f y：%.4f z：%.4f\n" RESET, it0->first, it0->second(0), it0->second(1), it0->second(2));
      }
      it0 = imu_pos_queue.erase(it0);
    }
  }
          
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
    auto params = _app->get_params();
    size_t num_unique_cameras = (params.state_options.num_cameras == 2) ? 1 : params.state_options.num_cameras;
    if (unique_cam_ids.size() == num_unique_cameras) {

      // Loop through our queue and see if we are able to process any of our camera measurements
      // We are able to process if we have at least one IMU measurement greater than the camera time
      double timestamp_imu_inC = message.timestamp - _app->get_state()->_calib_dt_CAMtoIMU->value()(0);
      while (!camera_queue.empty() && camera_queue.at(0).timestamp < timestamp_imu_inC) {
        auto rT0_1 = boost::posix_time::microsec_clock::local_time();
        double update_dt = 100.0 * (timestamp_imu_inC - camera_queue.at(0).timestamp);

        // if(!gnss_queue.empty()){
        //   std::lock_guard<std::mutex> lck(satnav_queue_mtx);
        //   while (!gnss_queue.empty() && gnss_queue.at(0).timestamp < camera_queue.at(0).timestamp)
        //   {
        //     if(gnss_queue.at(0).timestamp > _app->get_state()->_timestamp) {
        //       _app->feed_measurement_gnss(gnss_queue.at(0));   
        //     }
        //     gnss_queue.pop_front();
        //   }
        // }

        // if (!ins_gnss_queue.empty()) {
        //   std::lock_guard<std::mutex> lck(inspva_queue_mtx);
        //   while (!ins_gnss_queue.empty() && ins_gnss_queue.at(0).timestamp < camera_queue.at(0).timestamp) {
        //     // feed gnss data 
        //     if(ins_gnss_queue.at(0).timestamp > _app->get_state()->_timestamp) {
        //       _app->feed_measurement_gnss(ins_gnss_queue.at(0));    
        //     }
        //     ins_gnss_queue.pop_front();
        //   }
        // }
 
        PRINT_INFO(BLUE "ins_gnss队列的大小为：%d\n" RESET, ins_gnss_queue.size());
        if (!ins_gnss_queue.empty()) {
          std::lock_guard<std::mutex> lck(inspva_queue_mtx);
          while (!ins_gnss_queue.empty() && ins_gnss_queue.at(0).timestamp < camera_queue.at(0).timestamp) {
            // feed gnss data 
            // if(ins_gnss_queue.at(0).timestamp > _app->get_state()->_timestamp) {
            //   _app->feed_measurement_gnss(ins_gnss_queue.at(0));    
            // }
            
            PRINT_INFO(BLUE "ggp队列的大小为：%d\n" RESET, ggp_queue.size());
            if (!ggp_queue.empty()) {
              if (ggp_queue.at(0).timestamp == ins_gnss_queue.at(0).timestamp) {
                rover_fix_level = ggp_queue.at(0).rover_fix_level;
                _app->feed_measurement_gnss(ins_gnss_queue.at(0), rover_fix_level);
                ins_gnss_queue.pop_front();
                ggp_queue.pop_front();
              } else {
                _app->feed_measurement_gnss(ins_gnss_queue.at(0), rover_fix_level);
                ins_gnss_queue.pop_front();
              }
            } else {
              _app->feed_measurement_gnss(ins_gnss_queue.at(0), rover_fix_level);
              ins_gnss_queue.pop_front();
            }
          }
        }
        _app->feed_measurement_camera(camera_queue.at(0));

        ///手动失锁代码：直接去除GNSS输入
        /*
        // PRINT_INFO(GREEN "gnss队列的大小为：%d\n" RESET, gnss_queue.size());
        if (gnss_queue.size() > 1) {
          std::lock_guard<std::mutex> lck(satnav_queue_mtx);
          while (!gnss_queue.empty() && gnss_queue.at(0).timestamp < camera_queue.at(0).timestamp)
          {
            // feed gnss data, and simulate the status of gnss loselock
            _app->feed_measurement_gnss(gnss_queue.at(0), loselock_period);
            if (_app->initialized_gnss() && gnss_queue.at(0).timestamp >= _app->get_loselock_timestamp() && !loselock_begin_time_got) {
              loselock_begin_time = gnss_queue.at(0).timestamp;
              PRINT_INFO(RED "loselock_begin_time为：%.2f\n" RESET, loselock_begin_time);
              loselock_begin_time_got = true; 
              loselock_end_time_got = false;
            } else if (_app->initialized_gnss() && gnss_queue.at(1).timestamp > _app->get_loselock_timestamp() + loselock_period && !loselock_end_time_got) {
              loselock_end_time = gnss_queue.at(0).timestamp;
              PRINT_INFO(RED "loselock_end_time为：%.2f\n" RESET, loselock_end_time);
              loselock_end_time_got = true;
              loselock_begin_time_got = false;
            }
            gnss_queue.pop_front();
          }
        }

        _app->feed_measurement_camera(camera_queue.at(0));
        
        if (loselock_begin_time_got && !loselock_begin_dist_got ) {
          loselock_begin_dist = _app->get_distance();
          loselock_begin_dist_got = true;
          loselock_end_dist_got = false;
          PRINT_INFO(RED "loselock_begin_dist = %.2f (meters)\n", RESET, loselock_begin_dist);
        } else if (loselock_end_time_got && !loselock_end_dist_got) {
          loselock_end_dist = _app->get_distance();
          loselock_end_dist_got = true;
          loselock_begin_dist_got = false;
          PRINT_INFO(RED "loselock_end_dist = %.2f (meters)\n", RESET, loselock_end_dist);
          PRINT_INFO(RED "error_dist = %.2f (meters)\n", RESET, loselock_end_dist - loselock_begin_dist);
        }
        */

        visualize();
        camera_queue.pop_front();
        auto rT0_2 = boost::posix_time::microsec_clock::local_time();
        double time_total = (rT0_2 - rT0_1).total_microseconds() * 1e-6;
        PRINT_INFO(BLUE "[TIME]: %.4f seconds total (%.1f hz, %.2f ms behind)\n" RESET, time_total, 1.0 / time_total, update_dt);

        if(_app->initialized() && !_app->initialized_gnss()) {
          double timestamp = _app->get_state()->_timestamp;
          Eigen::Vector3d pos = _app->get_state()->_imu->pos();
          _app->feed_imu_pos(std::pair<double, Eigen::Vector3d>(timestamp, pos));
          PRINT_INFO(BLUE "!initialized_gnss, feed imu pos\n" RESET);
        }

        if(save_lla_file && _app->initialized()) {
          double timestamp = _app->get_state()->_timestamp;
          Eigen::Vector3d pos = _app->get_state()->_imu->pos();
          // PRINT_INFO(GREEN "保存lla文件时的pos值为：x：%.4f y：%.4f z：%.4f\n" RESET, pos(0), pos(1), pos(2));
          std::lock_guard<std::mutex> lck(fuse_pos_queue_mtx);
          fuse_pos.emplace_back(std::pair<double,Eigen::Vector3d>(timestamp,pos));
        }
      }
    }
    thread_update_running = false;
  });

  // If we are single threaded, then run single threaded
  // Otherwise detach this thread so it runs in the background!
  if (!_app->get_params().use_multi_threading_subs) {
    thread.join();
  } else {
    thread.detach();
  }
}

void ROS1Visualizer::callback_monocular(const sensor_msgs::ImageConstPtr &msg0, int cam_id0) {

  // Check if we should drop this image
  double timestamp = msg0->header.stamp.toSec();
  double time_delta = 1.0 / _app->get_params().track_frequency;
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
  if (_app->get_params().use_mask) {
    message.masks.push_back(_app->get_params().masks.at(cam_id0));
  } else {
    message.masks.push_back(cv::Mat::zeros(cv_ptr->image.rows, cv_ptr->image.cols, CV_8UC1));
  }

  // append it to our queue of images
  std::lock_guard<std::mutex> lck(camera_queue_mtx);
  camera_queue.push_back(message);
  std::sort(camera_queue.begin(), camera_queue.end());
}

void ROS1Visualizer::callback_stereo(const sensor_msgs::ImageConstPtr &msg0, const sensor_msgs::ImageConstPtr &msg1, int cam_id0,
                                     int cam_id1) {

  // Check if we should drop this image
  double timestamp = msg0->header.stamp.toSec();
  double time_delta = 1.0 / _app->get_params().track_frequency;
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
  if (_app->get_params().use_mask) {
    message.masks.push_back(_app->get_params().masks.at(cam_id0));
    message.masks.push_back(_app->get_params().masks.at(cam_id1));
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

void ROS1Visualizer::callback_satnav(const ov_msckf::satnavConstPtr &msg) {
  if(msg->pos_status==0) return;

  // convert into correct format
  ov_core::SatNavData message;
  message.timestamp = msg->header.header.stamp.toSec();

  message.week_num = msg->header.week_num;
  sat_week_num = message.week_num;
  if (sat_week_num == 0) {
    sat_week_num_got = false;
    return;
  } else {
    sat_week_num_got = true;
  }

  message.leap_sec = msg->header.leap_sec;
  sat_leap_sec = message.leap_sec;
  if (sat_leap_sec == 0) {
    sat_leap_sec_got = false;
    return;
  } else {
    sat_leap_sec_got = true;
  }

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
  message.sat_num_master = msg->sat_num_master;
  message.sat_num_slaver = msg->sat_num_slaver;

  // append it to our queue of gnss
  std::lock_guard<std::mutex> lck(satnav_queue_mtx);
  gnss_queue.push_back(message);
  std::sort(gnss_queue.begin(), gnss_queue.end());

  // pub gnss pos 
  if(!_app->initialized_gnss())
  {
    return;
  }

  std::vector<double> gnss_pub_init_lla = _app->get_init_lla();

  std::vector<double> lla(3);
  lla[0]=msg->latitude,lla[1]=msg->longitude,lla[2]=msg->altitude;

  std::vector<double> enu = lla2enu(lla,gnss_pub_init_lla);

  satnav_path.header.stamp = msg->header.header.stamp;
  satnav_path.header.frame_id = "global";
  geometry_msgs::PoseStamped curr_path;

  curr_path.header.stamp = msg->header.header.stamp;
  curr_path.header.frame_id = "global";

  Eigen::Vector3d tmp;
  tmp[0] = enu[0],tmp[1] = enu[1],tmp[2] = enu[2];
  tmp = _app->get_R_GNSStoI() * tmp + _app->get_t_GNSStoI();

  curr_path.pose.position.x = tmp[0];
  curr_path.pose.position.y = tmp[1];
  curr_path.pose.position.z = tmp[2];

  curr_path.pose.orientation.x = 0;
  curr_path.pose.orientation.y = 0;
  curr_path.pose.orientation.z = 0;
  curr_path.pose.orientation.w = 1;

  satnav_path.poses.push_back(curr_path);

  pub_satnav.publish(satnav_path);
}

/*
void ROS1Visualizer::callback_insnav(const ov_msckf::insnavConstPtr &msg) {
  if(msg->pos_status==0)
  {
    return;
  }
  // convert into correct format
  ov_core::InsNavData message;
  message.timestamp = msg->header.header.stamp.toSec();
  message.latitude = msg->latitude;
  message.longitude = msg->longitude;
  message.altitude = msg->altitude;
  message.ve = msg->vel_east;
  message.vn = msg->vel_north;
  message.vu = msg->vel_up;
  message.pitch = msg->pitch;
  message.roll = msg->roll;
  message.yaw = msg->yaw;
  message.pos_status = msg->pos_status;
  message.std_latitude = msg->std_latitude;
  message.std_longitude = msg->std_longitude;
  message.std_altitude = msg->std_altitude;
  // message.sat_accuracy_factor = msg->
  // message.sat_num_master = msg->
  // message.sat_num_slaver = msg->

  // append it to our queue of gnss
  std::lock_guard<std::mutex> lck(gnss_queue_mtx);
  gnss_queue.push_back(message);
  std::sort(gnss_queue.begin(), gnss_queue.end());

  //pub gnss pos 
  if(!_app->initialized_gnss())
  {
    return;
  }

  std::vector<double> gnss_pub_init_lla = _app->get_init_lla();

  std::vector<double> lla(3);
  lla[0]=msg->latitude,lla[1]=msg->longitude,lla[2]=msg->altitude;

  std::vector<double> enu = lla2enu(lla,gnss_pub_init_lla);

  insnav_path.header.stamp = msg->header.header.stamp;
  insnav_path.header.frame_id = "global";
  geometry_msgs::PoseStamped curr_path;

  curr_path.header.stamp = msg->header.header.stamp;
  curr_path.header.frame_id = "global";

  Eigen::Vector3d tmp;
  tmp[0] = enu[0],tmp[1] = enu[1],tmp[2] = enu[2];
  tmp = _app->get_R_GNSStoI() * tmp + _app->get_t_GNSStoI();

  curr_path.pose.position.x = tmp[0];
  curr_path.pose.position.y = tmp[1];
  curr_path.pose.position.z = tmp[2];

  curr_path.pose.orientation.x = 0;
  curr_path.pose.orientation.y = 0;
  curr_path.pose.orientation.z = 0;
  curr_path.pose.orientation.w = 1;

  insnav_path.poses.push_back(curr_path);

  pub_insnav.publish(insnav_path);
}
*/

void ROS1Visualizer::callback_ggp(const ov_msckf::ggpConstPtr &msg) {
  if (!unix_ref_imutime_got) return;
  ros::WallTime time = ros::WallTime(unix_ref_imutime);
  double year = time.toBoost().date().year();
  double month = time.toBoost().date().month();
  double day = time.toBoost().date().day();

  ov_core::GGPData message;
  message.timestamp = msg->utc_seconds;
  double hour = static_cast<int>(message.timestamp) / 10000;
  double minute = (static_cast<int>(message.timestamp) / 100) % 100;
  double second = std::fmod(message.timestamp, 100);

  double utc[6] = {year, month, day, hour, minute, second};
  gtime_t unix_time = epoch2time(utc);
  
  // 将time_t类型的unix时间转换成double类型，0代表unix参考时
  double time_double = std::difftime(unix_time.time, 0);
  // PRINT_INFO(GREEN "ggptime为：%.4f ggpsec为：%.4f\n" RESET, time_double, unix_time.sec);
  ros::Time ggp_ros_time;
  ggp_ros_time.fromSec(time_double + unix_time.sec);
  message.timestamp = ggp_ros_time.toSec();
  // PRINT_INFO(GREEN "message.timestamp为：%.2f\n" RESET, message.timestamp);

  message.rover_fix_level = msg->rover_fix_level;
  message.base_fix_level = msg->base_fix_level;
  ggp_rover_fix_level[message.timestamp] = message.rover_fix_level;
  ggp_base_fix_level[message.timestamp] = message.base_fix_level;

  if (ggp_base_fix_level.at(message.timestamp) == 0) {
    PRINT_INFO(GREEN "GGP分级协议基站状态：无效or当前为移动站模式\n" RESET);
    judge_ggp_rover_fix_level(ggp_rover_fix_level.at(message.timestamp));
    rover_fix_level_got = true;
  } else if (ggp_base_fix_level.at(message.timestamp) == 1) {
    PRINT_INFO(GREEN "GGP分级协议基站状态：excellent\n" RESET);
    judge_ggp_rover_fix_level(ggp_rover_fix_level.at(message.timestamp));
    rover_fix_level_got = true;
  } else if (ggp_base_fix_level.at(message.timestamp) == 2) {
    PRINT_INFO(GREEN "GGP分级协议基站状态：good\n" RESET);
    judge_ggp_rover_fix_level(ggp_rover_fix_level.at(message.timestamp));
    rover_fix_level_got = true;
  } else if (ggp_base_fix_level.at(message.timestamp) == 3) {
    PRINT_INFO(RED "GGP分级协议基站状态：bad\n" RESET);
    judge_ggp_rover_fix_level(ggp_rover_fix_level.at(message.timestamp));
    rover_fix_level_got = false;
  } else return;

  // double total_seconds = 5 * 24 * 3600 + hour * 3600 + minute * 60 + second;
  // gtime_t correct_time = gpst2time(2346, total_seconds);
  // std::cerr << "callback_ggp为:" << correct_time.time << "callback_ggp为" << correct_time.sec << std::endl;
  // PRINT_INFO(GREEN "转换后的utc时间为：%.4f\n" RESET, correct_time);

  // double integerPart;
  // double fractionalPart;
  message.lat = msg->lat;
  // PRINT_INFO(RED "callback_ggp为：x:%.8f\n" RESET, message.lat);
  // fractionalPart = std::modf(message.lat, &integerPart) / 0.6;
  // message.lat = integerPart + fractionalPart;
  // message.lat = static_cast<double>(static_cast<int>(message.lat)) + (message.lat - static_cast<int>(message.lat)) / 60.0;
  
  message.lon = msg->lon;
  // PRINT_INFO(RED "callback_ggp为：y:%.8f\n" RESET, message.lon);
  // fractionalPart = std::modf(message.lon, &integerPart) / 0.6;
  // message.lon = integerPart + fractionalPart;
  // message.lon = static_cast<double>(static_cast<int>(message.lon)) + (message.lon - static_cast<int>(message.lon)) / 60.0;
  
  message.alt = msg->alt + msg->undulation;
  // PRINT_INFO(RED "callback_ggp为：x:%.8f y:%.8f z:%.8f\n" RESET, message.lat, message.lon, message.alt);
  // fractionalPart = std::modf(message.alt, &integerPart) / 0.6;
  // message.alt = integerPart + fractionalPart;
  // message.undulation = msg->undulation;

  // PRINT_INFO(RED "callback_ggp为：x:%.10f y:%.10f z:%.10f\n" RESET, message.lat, message.lon, message.alt);

  // append it to our queue of ggp
  std::lock_guard<std::mutex> lck(ggp_queue_mtx);
  ggp_queue.push_back(message);
  std::sort(ggp_queue.begin(), ggp_queue.end());
}

void ROS1Visualizer::callback_inspva(const ov_msckf::inspvaConstPtr &msg) {

  if (!(sat_week_num_got && sat_leap_sec_got && rover_fix_level_got)) return;
  ov_core::InsPvaData message;
  message.timestamp = msg->gpstime;
  message.ins_status = msg->ins_status;
  message.ins_lat = msg->ins_lat;
  message.ins_lon = msg->ins_lon;
  message.ins_hgt = msg->ins_hgt;
  message.roll = msg->roll;
  message.pitch = msg->pitch;
  message.yaw = msg->yaw;
  message.gnss_status = msg->gnss_status;
  message.head_status = msg->head_status;
  message.gnss_lat = msg->gnss_lat;
  message.gnss_lon = msg->gnss_lon;
  message.gnss_hgt = msg->gnss_hgt;
  message.hdop = msg->hdop;
  message.sat_num_master = msg->satM;
  message.gyrox = msg->gyrox;
  message.gyroy = msg->gyroy;
  message.gyroz = msg->gyroz;
  message.accx = msg->accx;
  message.accy = msg->accy;
  message.accz = msg->accz;

  gtime_t correct_time = gpst2time(sat_week_num, message.timestamp);
  correct_time.time -= sat_leap_sec;
  ros::Time inspva_ros_time;
  inspva_ros_time.fromSec(static_cast<double>(correct_time.time) + correct_time.sec);
  message.timestamp = inspva_ros_time.toSec();
  // PRINT_INFO(GREEN "inspva中转换后的time为：%.2f\n" RESET, message.timestamp);

  // std::cerr << "callback_inspva函数中sat_week_num为：" << sat_week_num << std::endl;
  // std::cerr << "callback_inspva为:" << correct_time.time << "callback_inspva为" << correct_time.sec << std::endl;
  // PRINT_INFO(GREEN "callback_inspva为：x:%.12f y:%.12f z:%.12f\n" RESET, message.accx, message.accy, message.accz);
  // PRINT_INFO(GREEN "ins为：x:%.8f y:%.8f z:%.8f\n" RESET, message.ins_lat, message.ins_lon, message.ins_hgt);
  // PRINT_INFO(GREEN "gnss为：x:%.8f y:%.8f z:%.8f\n" RESET, message.gnss_lat, message.gnss_lon, message.gnss_hgt);

  // append it to our queue of gnss
  std::lock_guard<std::mutex> lck(inspva_queue_mtx);
  ins_gnss_queue.push_back(message);
  std::sort(ins_gnss_queue.begin(), ins_gnss_queue.end());

  // pub gnss pos 
  if(!_app->initialized_gnss()) {
    return;
  }

  std::vector<double> inspva_pub_init_lla = _app->get_init_lla();

  std::vector<double> lla(3);
  lla[0]=msg->ins_lat,lla[1]=msg->ins_lon,lla[2]=msg->ins_hgt;

  std::vector<double> enu = lla2enu(lla,inspva_pub_init_lla);

  inspva_path.header.stamp = inspva_ros_time;
  inspva_path.header.frame_id = "global";
  geometry_msgs::PoseStamped curr_path;

  curr_path.header.stamp = inspva_ros_time;
  curr_path.header.frame_id = "global";

  Eigen::Vector3d tmp;
  tmp[0] = enu[0], tmp[1] = enu[1], tmp[2] = enu[2];
  tmp = _app->get_R_GNSStoI() * tmp + _app->get_t_GNSStoI();

  curr_path.pose.position.x = tmp[0];
  curr_path.pose.position.y = tmp[1];
  curr_path.pose.position.z = tmp[2];

  curr_path.pose.orientation.x = 0;
  curr_path.pose.orientation.y = 0;
  curr_path.pose.orientation.z = 0;
  curr_path.pose.orientation.w = 1;

  inspva_path.poses.push_back(curr_path);

  pub_inspva.publish(inspva_path);
}

void ROS1Visualizer::publish_state() {

  // Get the current state
  std::shared_ptr<State> state = _app->get_state();

  // We want to publish in the IMU clock frame
  // The timestamp in the state will be the last camera time
  double t_ItoC = state->_calib_dt_CAMtoIMU->value()(0);
  double timestamp_inI = state->_timestamp + t_ItoC;

  // Create pose of IMU (note we use the bag time)
  geometry_msgs::PoseWithCovarianceStamped poseIinM;
  poseIinM.header.stamp = ros::Time(timestamp_inI);
  poseIinM.header.seq = poses_seq_imu;
  poseIinM.header.frame_id = "global";
  poseIinM.pose.pose.orientation.x = state->_imu->quat()(0);
  poseIinM.pose.pose.orientation.y = state->_imu->quat()(1);
  poseIinM.pose.pose.orientation.z = state->_imu->quat()(2);
  poseIinM.pose.pose.orientation.w = state->_imu->quat()(3);
  poseIinM.pose.pose.position.x = state->_imu->pos()(0);
  poseIinM.pose.pose.position.y = state->_imu->pos()(1);
  poseIinM.pose.pose.position.z = state->_imu->pos()(2);

  // Finally set the covariance in the message (in the order position then orientation as per ros convention)
  std::vector<std::shared_ptr<Type>> statevars;
  statevars.push_back(state->_imu->pose()->p());
  statevars.push_back(state->_imu->pose()->q());
  Eigen::Matrix<double, 6, 6> covariance_posori = StateHelper::get_marginal_covariance(_app->get_state(), statevars);
  for (int r = 0; r < 6; r++) {
    for (int c = 0; c < 6; c++) {
      poseIinM.pose.covariance[6 * r + c] = covariance_posori(r, c);
    }
  }
  pub_poseimu.publish(poseIinM);

  //=========================================================
  //=========================================================

  // Append to our pose vector
  geometry_msgs::PoseStamped posetemp;
  posetemp.header = poseIinM.header;
  posetemp.pose = poseIinM.pose.pose;
  poses_imu.push_back(posetemp);

  // Create our path (imu)
  // NOTE: We downsample the number of poses as needed to prevent rviz crashes
  // NOTE: https://github.com/ros-visualization/rviz/issues/1107
  nav_msgs::Path arrIMU;
  arrIMU.header.stamp = ros::Time::now();
  arrIMU.header.seq = poses_seq_imu;
  arrIMU.header.frame_id = "global";
  for (size_t i = 0; i < poses_imu.size(); i += std::floor((double)poses_imu.size() / 16384.0) + 1) {
    arrIMU.poses.push_back(poses_imu.at(i));
  }
  pub_pathimu.publish(arrIMU);

  // Move them forward in time
  poses_seq_imu++;
}

void ROS1Visualizer::publish_images() {

  // Return if we have already visualized
  if (_app->get_state() == nullptr)
    return;
  if (last_visualization_timestamp_image == _app->get_state()->_timestamp && _app->initialized())
    return;
  last_visualization_timestamp_image = _app->get_state()->_timestamp;

  // Check if we have subscribers
  if (it_pub_tracks.getNumSubscribers() == 0)
    return;

  // Get our image of history tracks
  cv::Mat img_history = _app->get_historical_viz_image();
  if (img_history.empty())
    return;

  // Create our message
  std_msgs::Header header;
  header.stamp = ros::Time::now();
  header.frame_id = "cam0";
  sensor_msgs::ImagePtr exl_msg = cv_bridge::CvImage(header, "bgr8", img_history).toImageMsg();

  // Publish
  it_pub_tracks.publish(exl_msg);
}

void ROS1Visualizer::publish_features() {

  // Check if we have subscribers
  if (pub_points_msckf.getNumSubscribers() == 0 && pub_points_slam.getNumSubscribers() == 0 && pub_points_aruco.getNumSubscribers() == 0 &&
      pub_points_sim.getNumSubscribers() == 0)
    return;

  // Get our good MSCKF features
  std::vector<Eigen::Vector3d> feats_msckf = _app->get_good_features_MSCKF();
  sensor_msgs::PointCloud2 cloud = ROSVisualizerHelper::get_ros_pointcloud(feats_msckf);
  pub_points_msckf.publish(cloud);

  // Get our good SLAM features
  std::vector<Eigen::Vector3d> feats_slam = _app->get_features_SLAM();
  sensor_msgs::PointCloud2 cloud_SLAM = ROSVisualizerHelper::get_ros_pointcloud(feats_slam);
  pub_points_slam.publish(cloud_SLAM);

  // Get our good ARUCO features
  std::vector<Eigen::Vector3d> feats_aruco = _app->get_features_ARUCO();
  sensor_msgs::PointCloud2 cloud_ARUCO = ROSVisualizerHelper::get_ros_pointcloud(feats_aruco);
  pub_points_aruco.publish(cloud_ARUCO);

  // Skip the rest of we are not doing simulation
  if (_sim == nullptr)
    return;

  // Get our good SIMULATION features
  std::vector<Eigen::Vector3d> feats_sim = _sim->get_map_vec();
  sensor_msgs::PointCloud2 cloud_SIM = ROSVisualizerHelper::get_ros_pointcloud(feats_sim);
  pub_points_sim.publish(cloud_SIM);
}

void ROS1Visualizer::publish_groundtruth() {

  // Our groundtruth state
  Eigen::Matrix<double, 17, 1> state_gt;

  // We want to publish in the IMU clock frame
  // The timestamp in the state will be the last camera time
  double t_ItoC = _app->get_state()->_calib_dt_CAMtoIMU->value()(0);
  double timestamp_inI = _app->get_state()->_timestamp + t_ItoC;

  // Check that we have the timestamp in our GT file [time(sec),q_GtoI,p_IinG,v_IinG,b_gyro,b_accel]
  if (_sim == nullptr && (gt_states.empty() || !DatasetReader::get_gt_state(timestamp_inI, state_gt, gt_states))) {
    return;
  }

  // Get the simulated groundtruth
  // NOTE: we get the true time in the IMU clock frame
  if (_sim != nullptr) {
    timestamp_inI = _app->get_state()->_timestamp + _sim->get_true_parameters().calib_camimu_dt;
    if (!_sim->get_state(timestamp_inI, state_gt))
      return;
  }

  // Get the GT and system state state
  Eigen::Matrix<double, 16, 1> state_ekf = _app->get_state()->_imu->value();

  // Create pose of IMU
  geometry_msgs::PoseStamped poseIinM;
  poseIinM.header.stamp = ros::Time(timestamp_inI);
  poseIinM.header.seq = poses_seq_gt;
  poseIinM.header.frame_id = "global";
  poseIinM.pose.orientation.x = state_gt(1, 0);
  poseIinM.pose.orientation.y = state_gt(2, 0);
  poseIinM.pose.orientation.z = state_gt(3, 0);
  poseIinM.pose.orientation.w = state_gt(4, 0);
  poseIinM.pose.position.x = state_gt(5, 0);
  poseIinM.pose.position.y = state_gt(6, 0);
  poseIinM.pose.position.z = state_gt(7, 0);
  pub_posegt.publish(poseIinM);

  // Append to our pose vector
  poses_gt.push_back(poseIinM);

  // Create our path (imu)
  // NOTE: We downsample the number of poses as needed to prevent rviz crashes
  // NOTE: https://github.com/ros-visualization/rviz/issues/1107
  nav_msgs::Path arrIMU;
  arrIMU.header.stamp = ros::Time::now();
  arrIMU.header.seq = poses_seq_gt;
  arrIMU.header.frame_id = "global";
  for (size_t i = 0; i < poses_gt.size(); i += std::floor((double)poses_gt.size() / 16384.0) + 1) {
    arrIMU.poses.push_back(poses_gt.at(i));
  }
  pub_pathgt.publish(arrIMU);

  // Move them forward in time
  poses_seq_gt++;

  // Publish our transform on TF
  tf::StampedTransform trans;
  trans.stamp_ = ros::Time::now();
  trans.frame_id_ = "global";
  trans.child_frame_id_ = "truth";
  tf::Quaternion quat(state_gt(1, 0), state_gt(2, 0), state_gt(3, 0), state_gt(4, 0));
  trans.setRotation(quat);
  tf::Vector3 orig(state_gt(5, 0), state_gt(6, 0), state_gt(7, 0));
  trans.setOrigin(orig);
  if (publish_global2imu_tf) {
    mTfBr->sendTransform(trans);
  }

  //==========================================================================
  //==========================================================================

  // Difference between positions
  double dx = state_ekf(4, 0) - state_gt(5, 0);
  double dy = state_ekf(5, 0) - state_gt(6, 0);
  double dz = state_ekf(6, 0) - state_gt(7, 0);
  double err_pos = std::sqrt(dx * dx + dy * dy + dz * dz);

  // Quaternion error
  Eigen::Matrix<double, 4, 1> quat_gt, quat_st, quat_diff;
  quat_gt << state_gt(1, 0), state_gt(2, 0), state_gt(3, 0), state_gt(4, 0);
  quat_st << state_ekf(0, 0), state_ekf(1, 0), state_ekf(2, 0), state_ekf(3, 0);
  quat_diff = quat_multiply(quat_st, Inv(quat_gt));
  double err_ori = (180 / M_PI) * 2 * quat_diff.block(0, 0, 3, 1).norm();

  //==========================================================================
  //==========================================================================

  // Get covariance of pose
  std::vector<std::shared_ptr<Type>> statevars;
  statevars.push_back(_app->get_state()->_imu->q());
  statevars.push_back(_app->get_state()->_imu->p());
  Eigen::Matrix<double, 6, 6> covariance = StateHelper::get_marginal_covariance(_app->get_state(), statevars);

  // Calculate NEES values
  // NOTE: need to manually multiply things out to make static asserts work
  // NOTE: https://github.com/rpng/open_vins/pull/226
  // NOTE: https://github.com/rpng/open_vins/issues/236
  // NOTE: https://gitlab.com/libeigen/eigen/-/issues/1664
  Eigen::Vector3d quat_diff_vec = quat_diff.block(0, 0, 3, 1);
  Eigen::Vector3d cov_vec = covariance.block(0, 0, 3, 3).inverse() * 2 * quat_diff.block(0, 0, 3, 1);
  double ori_nees = 2 * quat_diff_vec.dot(cov_vec);
  Eigen::Vector3d errpos = state_ekf.block(4, 0, 3, 1) - state_gt.block(5, 0, 3, 1);
  double pos_nees = errpos.transpose() * covariance.block(3, 3, 3, 3).inverse() * errpos;

  //==========================================================================
  //==========================================================================

  // Update our average variables
  if (!std::isnan(ori_nees) && !std::isnan(pos_nees)) {
    summed_mse_ori += err_ori * err_ori;
    summed_mse_pos += err_pos * err_pos;
    summed_nees_ori += ori_nees;
    summed_nees_pos += pos_nees;
    summed_number++;
  }

  // Nice display for the user
  PRINT_INFO(REDPURPLE "error to gt => %.3f, %.3f (deg,m) | rmse => %.3f, %.3f (deg,m) | called %d times\n" RESET, err_ori, err_pos,
             std::sqrt(summed_mse_ori / summed_number), std::sqrt(summed_mse_pos / summed_number), (int)summed_number);
  PRINT_INFO(REDPURPLE "nees => %.1f, %.1f (ori,pos) | avg nees = %.1f, %.1f (ori,pos)\n" RESET, ori_nees, pos_nees,
             summed_nees_ori / summed_number, summed_nees_pos / summed_number);

  //==========================================================================
  //==========================================================================
}

void ROS1Visualizer::publish_loopclosure_information() {

  // Get the current tracks in this frame
  double active_tracks_time1 = -1;
  double active_tracks_time2 = -1;
  std::unordered_map<size_t, Eigen::Vector3d> active_tracks_posinG;
  std::unordered_map<size_t, Eigen::Vector3d> active_tracks_uvd;
  cv::Mat active_cam0_image;
  _app->get_active_tracks(active_tracks_time1, active_tracks_posinG, active_tracks_uvd);
  _app->get_active_image(active_tracks_time2, active_cam0_image);
  if (active_tracks_time1 == -1)
    return;
  if (_app->get_state()->_clones_IMU.find(active_tracks_time1) == _app->get_state()->_clones_IMU.end())
    return;
  Eigen::Vector4d quat = _app->get_state()->_clones_IMU.at(active_tracks_time1)->quat();
  Eigen::Vector3d pos = _app->get_state()->_clones_IMU.at(active_tracks_time1)->pos();
  if (active_tracks_time1 != active_tracks_time2)
    return;

  // Default header
  std_msgs::Header header;
  header.stamp = ros::Time(active_tracks_time1);

  //======================================================
  // Check if we have subscribers for the pose odometry, camera intrinsics, or extrinsics
  if (pub_loop_pose.getNumSubscribers() != 0 || pub_loop_extrinsic.getNumSubscribers() != 0 ||
      pub_loop_intrinsics.getNumSubscribers() != 0) {

    // PUBLISH HISTORICAL POSE ESTIMATE
    nav_msgs::Odometry odometry_pose;
    odometry_pose.header = header;
    odometry_pose.header.frame_id = "global";
    odometry_pose.pose.pose.position.x = pos(0);
    odometry_pose.pose.pose.position.y = pos(1);
    odometry_pose.pose.pose.position.z = pos(2);
    odometry_pose.pose.pose.orientation.x = quat(0);
    odometry_pose.pose.pose.orientation.y = quat(1);
    odometry_pose.pose.pose.orientation.z = quat(2);
    odometry_pose.pose.pose.orientation.w = quat(3);
    pub_loop_pose.publish(odometry_pose);

    // PUBLISH IMU TO CAMERA0 EXTRINSIC
    // need to flip the transform to the IMU frame
    Eigen::Vector4d q_ItoC = _app->get_state()->_calib_IMUtoCAM.at(0)->quat();
    Eigen::Vector3d p_CinI = -_app->get_state()->_calib_IMUtoCAM.at(0)->Rot().transpose() * _app->get_state()->_calib_IMUtoCAM.at(0)->pos();
    nav_msgs::Odometry odometry_calib;
    odometry_calib.header = header;
    odometry_calib.header.frame_id = "imu";
    odometry_calib.pose.pose.position.x = p_CinI(0);
    odometry_calib.pose.pose.position.y = p_CinI(1);
    odometry_calib.pose.pose.position.z = p_CinI(2);
    odometry_calib.pose.pose.orientation.x = q_ItoC(0);
    odometry_calib.pose.pose.orientation.y = q_ItoC(1);
    odometry_calib.pose.pose.orientation.z = q_ItoC(2);
    odometry_calib.pose.pose.orientation.w = q_ItoC(3);
    pub_loop_extrinsic.publish(odometry_calib);

    // PUBLISH CAMERA0 INTRINSICS
    bool is_fisheye = (std::dynamic_pointer_cast<ov_core::CamEqui>(_app->get_params().camera_intrinsics.at(0)) != nullptr);
    sensor_msgs::CameraInfo cameraparams;
    cameraparams.header = header;
    cameraparams.header.frame_id = "cam0";
    cameraparams.distortion_model = is_fisheye ? "equidistant" : "plumb_bob";
    Eigen::VectorXd cparams = _app->get_state()->_cam_intrinsics.at(0)->value();
    cameraparams.D = {cparams(4), cparams(5), cparams(6), cparams(7)};
    cameraparams.K = {cparams(0), 0, cparams(2), 0, cparams(1), cparams(3), 0, 0, 1};
    pub_loop_intrinsics.publish(cameraparams);
  }

  //======================================================
  // PUBLISH FEATURE TRACKS IN THE GLOBAL FRAME OF REFERENCE
  if (pub_loop_point.getNumSubscribers() != 0) {

    // Construct the message
    sensor_msgs::PointCloud point_cloud;
    point_cloud.header = header;
    point_cloud.header.frame_id = "global";
    for (const auto &feattimes : active_tracks_posinG) {

      // Get this feature information
      size_t featid = feattimes.first;
      Eigen::Vector3d uvd = Eigen::Vector3d::Zero();
      if (active_tracks_uvd.find(featid) != active_tracks_uvd.end()) {
        uvd = active_tracks_uvd.at(featid);
      }
      Eigen::Vector3d pFinG = active_tracks_posinG.at(featid);

      // Push back 3d point
      geometry_msgs::Point32 p;
      p.x = pFinG(0);
      p.y = pFinG(1);
      p.z = pFinG(2);
      point_cloud.points.push_back(p);

      // Push back the uv_norm, uv_raw, and feature id
      // NOTE: we don't use the normalized coordinates to save time here
      // NOTE: they will have to be re-normalized in the loop closure code
      sensor_msgs::ChannelFloat32 p_2d;
      p_2d.values.push_back(0);
      p_2d.values.push_back(0);
      p_2d.values.push_back(uvd(0));
      p_2d.values.push_back(uvd(1));
      p_2d.values.push_back(featid);
      point_cloud.channels.push_back(p_2d);
    }
    pub_loop_point.publish(point_cloud);
  }

  //======================================================
  // Depth images of sparse points and its colorized version
  if (it_pub_loop_img_depth.getNumSubscribers() != 0 || it_pub_loop_img_depth_color.getNumSubscribers() != 0) {

    // Create the images we will populate with the depths
    std::pair<int, int> wh_pair = {active_cam0_image.cols, active_cam0_image.rows};
    cv::Mat depthmap = cv::Mat::zeros(wh_pair.second, wh_pair.first, CV_16UC1);
    cv::Mat depthmap_viz = active_cam0_image;

    // Loop through all points and append
    for (const auto &feattimes : active_tracks_uvd) {

      // Get this feature information
      size_t featid = feattimes.first;
      Eigen::Vector3d uvd = active_tracks_uvd.at(featid);

      // Skip invalid points
      double dw = 4;
      if (uvd(0) < dw || uvd(0) > wh_pair.first - dw || uvd(1) < dw || uvd(1) > wh_pair.second - dw) {
        continue;
      }

      // Append the depth
      // NOTE: scaled by 1000 to fit the 16U
      // NOTE: access order is y,x (stupid opencv convention stuff)
      depthmap.at<uint16_t>((int)uvd(1), (int)uvd(0)) = (uint16_t)(1000 * uvd(2));

      // Taken from LSD-SLAM codebase segment into 0-4 meter segments:
      // https://github.com/tum-vision/lsd_slam/blob/d1e6f0e1a027889985d2e6b4c0fe7a90b0c75067/lsd_slam_core/src/util/globalFuncs.cpp#L87-L96
      float id = 1.0f / (float)uvd(2);
      float r = (0.0f - id) * 255 / 1.0f;
      if (r < 0)
        r = -r;
      float g = (1.0f - id) * 255 / 1.0f;
      if (g < 0)
        g = -g;
      float b = (2.0f - id) * 255 / 1.0f;
      if (b < 0)
        b = -b;
      uchar rc = r < 0 ? 0 : (r > 255 ? 255 : r);
      uchar gc = g < 0 ? 0 : (g > 255 ? 255 : g);
      uchar bc = b < 0 ? 0 : (b > 255 ? 255 : b);
      cv::Scalar color(255 - rc, 255 - gc, 255 - bc);

      // Small square around the point (note the above bound check needs to take into account this width)
      cv::Point p0(uvd(0) - dw, uvd(1) - dw);
      cv::Point p1(uvd(0) + dw, uvd(1) + dw);
      cv::rectangle(depthmap_viz, p0, p1, color, -1);
    }

    // Create our messages
    header.frame_id = "cam0";
    sensor_msgs::ImagePtr exl_msg1 = cv_bridge::CvImage(header, sensor_msgs::image_encodings::TYPE_16UC1, depthmap).toImageMsg();
    it_pub_loop_img_depth.publish(exl_msg1);
    header.stamp = ros::Time::now();
    header.frame_id = "cam0";
    sensor_msgs::ImagePtr exl_msg2 = cv_bridge::CvImage(header, "bgr8", depthmap_viz).toImageMsg();
    it_pub_loop_img_depth_color.publish(exl_msg2);
  }
}
