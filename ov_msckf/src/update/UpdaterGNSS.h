#ifndef OV_MSCKF_UPDATER_GNSS_H
#define OV_MSCKF_UPDATER_GNSS_H

#include <memory>
#include "utils/sensor_data.h"

namespace ov_msckf {

class State;
class Propagator;

class UpdaterGNSS {

public:
    UpdaterGNSS(std::shared_ptr<Propagator> prop);

    /**
    * @brief Feed function for inertial data
    * @param message Contains our timestamp and inertial information
    * @param oldest_time Time that we can discard measurements before
    */
    void feed_imu(const ov_core::ImuData &message, double oldest_time = -1) {

        // Append it to our vector
        imu_data.emplace_back(message);

        // Sort our imu data (handles any out of order measurements)
        // std::sort(imu_data.begin(), imu_data.end(), [](const IMUDATA i, const IMUDATA j) {
        //    return i.timestamp < j.timestamp;
        //});

        // Clean old measurements
        // std::cout << "ZVUPT: imu_data.size() " << imu_data.size() << std::endl;
        clean_old_imu_measurements(oldest_time - 0.10);
    }

  /**
   * @brief This will remove any IMU measurements that are older then the given measurement time
   * @param oldest_time Time that we can discard measurements before (in IMU clock)
   */
  void clean_old_imu_measurements(double oldest_time) {
    if (oldest_time < 0)
      return;
    auto it0 = imu_data.begin();
    while (it0 != imu_data.end()) {
      if (it0->timestamp < oldest_time) {
        it0 = imu_data.erase(it0);
      } else {
        it0++;
      }
    }
  }

  /**
   * @brief Will first detect if the gnss pos is valid, then will update.
   * @param state State of the filter
   * @param gnss_data gnss data
   * @param init_lla lat,lon,alt transform to enu reference 
   * @param init_R_GNSStoI gnss enu position tranform Matrix to imu frame
   * @param init_t_GNSStoI gnss enu position translation to imu frame
   * @return True if the system is fuse gnss data
   */
  bool try_update(std::shared_ptr<State> state, const ov_core::GnssData &gnss_data,const Eigen::Vector3d &init_lla,
                const Eigen::Matrix3d &init_R_GNSStoI,const Eigen::Vector3d &init_t_GNSStoI);
protected:
    /// Our propagator!
    std::shared_ptr<Propagator> _prop;
    /// Our history of IMU messages (time, angular, linear)
    std::vector<ov_core::ImuData> imu_data;

};

}// namespace ov_msckf

#endif