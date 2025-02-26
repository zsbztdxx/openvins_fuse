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

#ifndef OV_CORE_SENSOR_DATA_H
#define OV_CORE_SENSOR_DATA_H

#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include <vector>

namespace ov_core {

/**
 * @brief Struct for a single imu measurement (time, wm, am)
 */
struct ImuData {

  /// Timestamp of the reading
  double timestamp;

  /// Gyroscope reading, angular velocity (rad/s)
  Eigen::Matrix<double, 3, 1> wm;

  /// Accelerometer reading, linear acceleration (m/s^2)
  Eigen::Matrix<double, 3, 1> am;

  /// Sort function to allow for using of STL containers
  bool operator<(const ImuData &other) const { return timestamp < other.timestamp; }
};

/**
 * @brief Struct for a collection of camera measurements.
 *
 * For each image we have a camera id and timestamp that it occured at.
 * If there are multiple cameras we will treat it as pair-wise stereo tracking.
 */
struct CameraData {

  /// Timestamp of the reading
  double timestamp;

  /// Camera ids for each of the images collected
  std::vector<int> sensor_ids;

  /// Raw image we have collected for each camera
  std::vector<cv::Mat> images;

  /// Tracking masks for each camera we have
  std::vector<cv::Mat> masks;

  /// Sort function to allow for using of STL containers
  bool operator<(const CameraData &other) const {
    if (timestamp == other.timestamp) {
      int id = *std::min_element(sensor_ids.begin(), sensor_ids.end());
      int id_other = *std::min_element(other.sensor_ids.begin(), other.sensor_ids.end());
      return id < id_other;
    } else {
      return timestamp < other.timestamp;
    }
  }
};

/**
 * @brief Struct for a satnav measurement
 */
struct SatNavData {

  /// Timestamp of the reading
  double timestamp;

  int week_num, leap_sec;

  /// int status
  int status_pos;

  /// latitude,longitude,altitude
  double latitude,longitude,altitude;

  /// velocity of enu
  double ve,vn,vu;

  /// std
  double std_latitude,std_longitude,std_altitude,std_ve,std_vn,std_vu;

  /// num of sat
  double sat_num_master, sat_num_slaver;

  /// Sort function to allow for using of STL containers
  bool operator<(const SatNavData &other) const { return timestamp < other.timestamp; }
};

/**
 * @brief Struct for a insnav measurement
 */
struct InsNavData {

  /// Timestamp of the reading
  double timestamp;

  /// ins status
  int ins_status;

  /// latitude,longitude,altitude
  double latitude, longitude, altitude;

  /// velocity of enu
  double ve, vn, vu;

  /// heading angle
  double pitch, roll, yaw;

  /// nav and pos status
  int nav_status, pos_status;

  /// sat accuracy factor
  double sat_accuracy_factor;

  /// num of sat
  double sat_num_master, sat_num_slaver;

  /// std but unbelievable
  double std_latitude, std_longitude, std_altitude, std_ve, std_vn, std_vu, std_roll, std_pitch, std_yaw;

  /// Sort function to allow for using of STL containers
  bool operator<(const InsNavData &other) const { return timestamp < other.timestamp; }
};

/**
 * @brief Struct for a inspvaa measurement
 */
struct InsPvaData {

  /// Timestamp of the reading
  double timestamp;

  int ins_status;

  /// latitude,longitude,altitude
  double ins_lat, ins_lon, ins_hgt;

  /// velocity of enu
  double ve, vn, vu;

  /// heading angle
  double roll, pitch, yaw;

  int gnss_status, head_status;

  double baseline;
  
  double gnss_lat, gnss_lon, gnss_hgt, gnss_normv;

  /// horizontal dilution of precision
  double hdop;

  /// num of sat
  double sat_num_master, sat_num_slaver;
  
  double odomv;

  int odomflag;

  double gyrox, gyroy, gyroz;

  double accx, accy, accz, accstdval;

  /// Sort function to allow for using of STL containers
  bool operator<(const InsPvaData &other) const { return timestamp < other.timestamp; }
};

struct GGPData {
  double timestamp;

  double lat, lon, alt;

  int gps_qual, num_sats;

  double hdop;

  double undulation;

  int diff_age, station_id;

  int rover_fix_level, base_fix_level;

  bool operator<(const GGPData &other) const { return timestamp < other.timestamp; }
};

} // namespace ov_core

#endif // OV_CORE_SENSOR_DATA_H