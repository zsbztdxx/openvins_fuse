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

#include "InertialInitializer.h"

#include "dynamic/DynamicInitializer.h"
#include "static/StaticInitializer.h"

#include "feat/FeatureHelper.h"
#include "types/Type.h"
#include "utils/colors.h"
#include "utils/print.h"
#include "utils/quat_ops.h"
#include "utils/sensor_data.h"
#include "utils/pos_transform.h"
#include "utils/Coosys.h"

using namespace ov_core;
using namespace ov_type;
using namespace ov_init;

InertialInitializer::InertialInitializer(InertialInitializerOptions &params_, std::shared_ptr<ov_core::FeatureDatabase> db)
    : params(params_), _db(db) {

  // Vector of our IMU data
  imu_data = std::make_shared<std::vector<ov_core::ImuData>>();

  // Vector of our GNSS data
  gnss_data = std::make_shared<std::vector<ov_core::SatNavData>>();
  ins_gnss_data = std::make_shared<std::vector<ov_core::InsPvaData>>();

  // Vector of our imu pos data
  imu_pos_vec = std::make_shared<std::vector<std::pair<double,Eigen::Vector3d>>>();

  // Create initializers
  init_static = std::make_shared<StaticInitializer>(params, _db, imu_data);
  init_dynamic = std::make_shared<DynamicInitializer>(params, _db, imu_data);
}

void InertialInitializer::feed_imu(const ov_core::ImuData &message, double oldest_time) {

  // Append it to our vector
  imu_data->emplace_back(message);

  // Sort our imu data (handles any out of order measurements)
  // std::sort(imu_data->begin(), imu_data->end(), [](const IMUDATA i, const IMUDATA j) {
  //    return i.timestamp < j.timestamp;
  //});

  // Loop through and delete imu messages that are older than our requested time
  // std::cout << "INIT: imu_data.size() " << imu_data->size() << std::endl;
  if (oldest_time != -1) {
    auto it0 = imu_data->begin();
    while (it0 != imu_data->end()) {
      if (it0->timestamp < oldest_time) {
        it0 = imu_data->erase(it0);
      } else {
        it0++;
      }
    }
  }
}

void InertialInitializer::feed_gnss(const ov_core::SatNavData &message, double oldest_time) {
  // Append it to our vector
  gnss_data->emplace_back(message);

  // Loop through and delete gnss messages that are older than our requested time
  // std::cout << "INIT: gnss_data.size() " << gnss_data->size() << std::endl;
  if (oldest_time != -1) {
    auto it0 = gnss_data->begin();
    while (it0 != gnss_data->end()) {
      if (it0->timestamp < oldest_time) {
        it0 = gnss_data->erase(it0);
      } else {
        it0++;
      }
    }
  }
}

void InertialInitializer::feed_gnss(const ov_core::InsPvaData &message, double oldest_time) {
  // Append it to our vector
  ins_gnss_data->emplace_back(message);

  // Loop through and delete gnss messages that are older than our requested time
  // std::cout << "INIT: gnss_data.size() " << gnss_data->size() << std::endl;
  if (oldest_time != -1) {
    auto it0 = ins_gnss_data->begin();
    while (it0 != ins_gnss_data->end()) {
      if (it0->timestamp < oldest_time) {
        it0 = ins_gnss_data->erase(it0);
      } else {
        it0++;
      }
    }
  }
}

void InertialInitializer::feed_imu_pos(const std::pair<double, Eigen::Vector3d> &pos, double oldest_time){
  // Append it to our vector
  imu_pos_vec->emplace_back(pos);
  if (oldest_time != -1) {
    auto it0 = imu_pos_vec->begin();
    while (it0 != imu_pos_vec->end()) {
      if (it0->first < oldest_time) {
        it0 = imu_pos_vec->erase(it0);
      } else {
        it0++;
      }
    }
  } else {
    oldest_time = pos.first - params.init_gnss_window_time;
    auto it0 = imu_pos_vec->begin();
    while (it0 != imu_pos_vec->end()) {
      if (it0->first < oldest_time) {
        it0 = imu_pos_vec->erase(it0);
      } else {
        it0++;
      }
    }
  }
}

bool InertialInitializer::initialize(double &timestamp, Eigen::MatrixXd &covariance, std::vector<std::shared_ptr<ov_type::Type>> &order,
                                     std::shared_ptr<ov_type::IMU> t_imu, bool wait_for_jerk) {

  // Get the newest and oldest timestamps we will try to initialize between!
  double newest_cam_time = -1;
  for (auto const &feat : _db->get_internal_data()) {
    for (auto const &camtimepair : feat.second->timestamps) {
      for (auto const &time : camtimepair.second) {
        newest_cam_time = std::max(newest_cam_time, time);
      }
    }
  }
  double oldest_time = newest_cam_time - params.init_window_time - 0.10;
  if (newest_cam_time < 0 || oldest_time < 0) {
    return false;
  }

  // Remove all measurements that are older then our initialization window
  // Then we will try to use all features that are in the feature database!
  _db->cleanup_measurements(oldest_time);
  auto it_imu = imu_data->begin();
  while (it_imu != imu_data->end() && it_imu->timestamp < oldest_time + params.calib_camimu_dt) {
    it_imu = imu_data->erase(it_imu);
  }

  // Compute the disparity of the system at the current timestep
  // If disparity is zero or negative we will always use the static initializer
  bool disparity_detected_moving_1to0 = false;
  bool disparity_detected_moving_2to1 = false;
  if (params.init_max_disparity > 0) {

    // Get the disparity statistics from this image to the previous
    // Only compute the disparity for the oldest half of the initialization period
    double newest_time_allowed = newest_cam_time - 0.5 * params.init_window_time;
    int num_features0 = 0;
    int num_features1 = 0;
    double avg_disp0, avg_disp1;
    double var_disp0, var_disp1;
    FeatureHelper::compute_disparity(_db, avg_disp0, var_disp0, num_features0, newest_time_allowed);
    FeatureHelper::compute_disparity(_db, avg_disp1, var_disp1, num_features1, newest_cam_time, newest_time_allowed);

    // Return if we can't compute the disparity
    int feat_thresh = 15;
    if (num_features0 < feat_thresh || num_features1 < feat_thresh) {
      PRINT_WARNING(YELLOW "[init]: not enough feats to compute disp: %d,%d < %d\n" RESET, num_features0, num_features1, feat_thresh);
      return false;
    }

    // Check if it passed our check!
    PRINT_INFO(YELLOW "[init]: disparity is %.3f,%.3f (%.2f thresh)\n" RESET, avg_disp0, avg_disp1, params.init_max_disparity);
    disparity_detected_moving_1to0 = (avg_disp0 > params.init_max_disparity);
    disparity_detected_moving_2to1 = (avg_disp1 > params.init_max_disparity);
  }

  // Use our static initializer!
  // CASE1: if our disparity says we were static in last window and have moved in the newest, we have a jerk
  // CASE2: if both disparities are below the threshold, then the platform has been stationary during both periods
  bool has_jerk = (!disparity_detected_moving_1to0 && disparity_detected_moving_2to1);
  bool is_still = (!disparity_detected_moving_1to0 && !disparity_detected_moving_2to1);
  if (((has_jerk && wait_for_jerk) || (is_still && !wait_for_jerk)) && params.init_imu_thresh > 0.0) {
    PRINT_DEBUG(GREEN "[init]: USING STATIC INITIALIZER METHOD!\n" RESET);
    return init_static->initialize(timestamp, covariance, order, t_imu, wait_for_jerk);
  } else if (params.init_dyn_use && !is_still) {
    PRINT_DEBUG(GREEN "[init]: USING DYNAMIC INITIALIZER METHOD!\n" RESET);
    std::map<double, std::shared_ptr<ov_type::PoseJPL>> _clones_IMU;
    std::unordered_map<size_t, std::shared_ptr<ov_type::Landmark>> _features_SLAM;
    return init_dynamic->initialize(timestamp, covariance, order, t_imu, _clones_IMU, _features_SLAM);
  } else {
    std::string msg = (has_jerk) ? "" : "no accel jerk detected";
    msg += (has_jerk || is_still) ? "" : ", ";
    msg += (is_still) ? "" : "platform moving too much";
    PRINT_INFO(YELLOW "[init]: failed static init: %s\n" RESET, msg.c_str());
  }
  return false;
}


bool InertialInitializer::initialize_gnss(Eigen::Matrix3d &R_GNSStoI, Eigen::Vector3d &tran,std::vector<double> &init_lla,
                                    std::map<double, std::shared_ptr<ov_type::PoseJPL >> &clone_imu,double dt_CAMtoIMU)
{
  /*if(gnss_data->size()<2 || clone_imu.size()<2)
  {
    return false;
  }*/
  if(gnss_data->size()<2 || imu_pos_vec->size()<2)
  {
    return false;
  }

  std::vector<double> lla0(3);
  double dt = DBL_MAX;
  for(size_t i=0;i<gnss_data->size();i++){
    if(fabs(gnss_data->at(i).timestamp-imu_pos_vec->at(0).first)<dt){
      lla0[0] = gnss_data->at(i).latitude;
      lla0[1] = gnss_data->at(i).longitude;
      lla0[2] = gnss_data->at(i).altitude;
    }
  }

  std::vector<Eigen::Vector3d> imu_pos,gnss_pos;
  for(auto imu:*imu_pos_vec)
  {
    double ti = imu.first;

    std::pair<double,std::vector<double>> pre_lla,next_lla;
    pre_lla.first = DBL_MAX,next_lla.first = DBL_MAX;
    for(size_t i=0;i<gnss_data->size();i++)
    {
      // if(gnss_data->at(i).status_pos!=4){
      //   continue;
      // }
      double tg = gnss_data->at(i).timestamp - dt_CAMtoIMU;
      std::vector<double> lla(3);
      lla[0] = gnss_data->at(i).latitude;
      lla[1] = gnss_data->at(i).longitude;
      lla[2] = gnss_data->at(i).altitude;
      double dt = tg - ti;
      if(dt < 0 && fabs(dt) < fabs(pre_lla.first - ti))
      {
        pre_lla.first = tg;
        pre_lla.second = lla;
      } else if (dt > 0 && fabs(dt) < fabs(next_lla.first - ti))
      {
        next_lla.first = tg;
        next_lla.second = lla;
      }
    }
    if( (next_lla.first - pre_lla.first < 1e-6) || (next_lla.first - pre_lla.first > 0.2)){
      continue;
    }
    std::vector<double> pre_enu = lla2enu(pre_lla.second,lla0);
    std::vector<double> next_enu = lla2enu(next_lla.second,lla0);
    std::pair<double,Eigen::Vector3d> pos0,pos1;
    pos0.first = pre_lla.first, pos1.first = next_lla.first;
    pos0.second[0] = pre_enu[0],pos0.second[1] = pre_enu[1],pos0.second[2] = pre_enu[2];
    pos1.second[0] = next_enu[0],pos1.second[1] = next_enu[1],pos1.second[2] = next_enu[2];
    Eigen::Vector3d enu = linear_interpolation_enu(pos0,pos1,ti);
    gnss_pos.push_back(enu);
    //imu_pos.push_back(imu.second->pos());
    imu_pos.push_back(imu.second);
    //PRINT_INFO(YELLOW "pre gnss time: %f\n" RESET, pre_lla.first);
    //PRINT_INFO(YELLOW "clone imu time: %f\n" RESET, ti);
    //PRINT_INFO(YELLOW "next gnss time: %f\n" RESET, next_lla.first);
  }
  if(gnss_pos.size()<3){
    PRINT_INFO(YELLOW "not enough gnss data to init\n" RESET);
    return false;
  }
  double s = 0;
  for(size_t i=0;i<imu_pos.size()-1;i++)
  {
    Eigen::Vector3d ds = imu_pos[i+1] - imu_pos[i];
    s += std::sqrt(ds[0]*ds[0]+ds[1]*ds[1]+ds[2]*ds[2]);
  }
  if(s<params.init_gnss_min_move){
    PRINT_INFO(YELLOW "not enough move to init gnss\n" RESET);
    return false;
  }
  for(size_t i=0;i<imu_pos.size();i++)
  {
    PRINT_INFO(YELLOW "imu pos[%d]:(%f,%f,%f),gnss pos[%d]:(%f,%f,%f)\n" RESET, i,imu_pos[i][0],imu_pos[i][1],imu_pos[i][2],i,gnss_pos[i][0],gnss_pos[i][1],gnss_pos[i][2]);
  }

  Eigen::Matrix3d RWC;
  Eigen::Vector3d TWC;
  double SWC;

  if (!getRTWC(gnss_pos, imu_pos, RWC, TWC, SWC))
  {
    PRINT_INFO(YELLOW "gnss and vio path align falied\n" RESET);
    return false;
  }

  Eigen::Vector3d n = RWC.col(0);

  double yaw = atan2(n(1), n(0));

  PRINT_INFO(YELLOW "scale:%f,yaw:%f,t:(%f,%f,%f)\n" RESET,SWC,yaw / M_PI * 180.0,TWC(0),TWC(1),TWC(2));
  PRINT_INFO(YELLOW "imu_pos_vec size: %d\n" RESET, imu_pos_vec->size());

  R_GNSStoI = RWC.transpose();
  if(SWC<0){
    R_GNSStoI = -R_GNSStoI;
  }
  tran = R_GNSStoI * (-TWC);
  init_lla = lla0;

  return true;
}

bool InertialInitializer::initialize_ins_gnss(Eigen::Matrix3d &R_GNSStoI, Eigen::Vector3d &tran,std::vector<double> &init_lla,
                                    std::map<double, std::shared_ptr<ov_type::PoseJPL >> &clone_imu,double dt_CAMtoIMU) {
  
  /*if(gnss_data->size()<2 || clone_imu.size()<2)
  {
    return false;
  }*/
  if(ins_gnss_data->size()<2 || imu_pos_vec->size()<2)
  {
    return false;
  }

  std::vector<double> lla0(3);
  double dt = DBL_MAX;
  for(size_t i = 0; i < ins_gnss_data->size(); i++){
    if(fabs(ins_gnss_data->at(i).timestamp - imu_pos_vec->at(0).first) < dt) {
      lla0[0] = ins_gnss_data->at(i).ins_lat;
      lla0[1] = ins_gnss_data->at(i).ins_lon;
      lla0[2] = ins_gnss_data->at(i).ins_hgt;
    }
  }

  std::vector<Eigen::Vector3d> imu_pos, gnss_pos;
  for(auto &imu : *imu_pos_vec)
  {
    double ti = imu.first;

    std::pair<double, std::vector<double>> pre_lla, next_lla;
    pre_lla.first = DBL_MAX, next_lla.first = DBL_MAX;
    for(size_t i = 0; i < ins_gnss_data->size(); i++)
    {
      // if(ins_gnss_data->at(i).gnss_status != 4){
      //   continue;
      // }
      double tg = ins_gnss_data->at(i).timestamp - dt_CAMtoIMU;
      std::vector<double> lla(3);
      lla[0] = ins_gnss_data->at(i).ins_lat;
      lla[1] = ins_gnss_data->at(i).ins_lon;
      lla[2] = ins_gnss_data->at(i).ins_hgt;
      double dt = tg - ti;
      if (dt < 0 && fabs(dt) < fabs(pre_lla.first - ti))
      {
        pre_lla.first = tg;
        pre_lla.second = lla;
      } else if (dt > 0 && fabs(dt) < fabs(next_lla.first - ti))
      {
        next_lla.first = tg;
        next_lla.second = lla;
      }
    }
    if((next_lla.first - pre_lla.first < 1e-6) || (next_lla.first - pre_lla.first > 0.2)){
      continue;
    }
    std::vector<double> pre_enu = lla2enu(pre_lla.second, lla0);
    std::vector<double> next_enu = lla2enu(next_lla.second, lla0);
    std::pair<double, Eigen::Vector3d> pos0, pos1;
    pos0.first = pre_lla.first, pos1.first = next_lla.first;
    pos0.second[0] = pre_enu[0], pos0.second[1] = pre_enu[1], pos0.second[2] = pre_enu[2];
    pos1.second[0] = next_enu[0], pos1.second[1] = next_enu[1], pos1.second[2] = next_enu[2];
    Eigen::Vector3d enu = linear_interpolation_enu(pos0, pos1, ti);
    gnss_pos.push_back(enu);
    //imu_pos.push_back(imu.second->pos());
    imu_pos.push_back(imu.second);
    //PRINT_INFO(YELLOW "pre gnss time: %f\n" RESET, pre_lla.first);
    //PRINT_INFO(YELLOW "clone imu time: %f\n" RESET, ti);
    //PRINT_INFO(YELLOW "next gnss time: %f\n" RESET, next_lla.first);
  }
  if(gnss_pos.size() < 3) {
    PRINT_INFO(YELLOW "not enough gnss data to init\n" RESET);
    return false;
  }
  double s = 0;
  for(size_t i=0;i<imu_pos.size()-1;i++)
  {
    Eigen::Vector3d ds = imu_pos[i+1] - imu_pos[i];
    s += std::sqrt(ds[0]*ds[0]+ds[1]*ds[1]+ds[2]*ds[2]);
  }
  if(s<params.init_gnss_min_move){
    PRINT_INFO(YELLOW "not enough move to init gnss\n" RESET);
    return false;
  }
  for(size_t i=0;i<imu_pos.size();i++)
  {
    PRINT_INFO(YELLOW "imu pos[%d]:(%f,%f,%f),gnss pos[%d]:(%f,%f,%f)\n" RESET, i,imu_pos[i][0],imu_pos[i][1],imu_pos[i][2],i,gnss_pos[i][0],gnss_pos[i][1],gnss_pos[i][2]);
  }

  Eigen::Matrix3d RWC;
  Eigen::Vector3d TWC;
  double SWC;

  if (!getRTWC(gnss_pos, imu_pos, RWC, TWC, SWC))
  {
    PRINT_INFO(YELLOW "gnss and vio path align falied\n" RESET);
    return false;
  }

  Eigen::Vector3d n = RWC.col(0);

  double yaw = atan2(n(1), n(0));

  PRINT_INFO(YELLOW "scale:%f,yaw:%f,t:(%f,%f,%f)\n" RESET,SWC,yaw / M_PI * 180.0,TWC(0),TWC(1),TWC(2));
  PRINT_INFO(YELLOW "imu_pos_vec size: %d\n" RESET, imu_pos_vec->size());

  R_GNSStoI = RWC.transpose();
  if(SWC<0){
    R_GNSStoI = -R_GNSStoI;
  }
  tran = R_GNSStoI * (-TWC);
  init_lla = lla0;

  return true;
}