#ifndef OV_MSCKF_UPDATER_GNSS_H
#define OV_MSCKF_UPDATER_GNSS_H

#include <memory>
#include "utils/sensor_data.h"
#include "utils/NoiseManager.h"

namespace ov_msckf {

class State;
class Propagator;

class UpdaterGNSS {

public:
    UpdaterGNSS(NoiseManager &noises, std::shared_ptr<Propagator> prop);

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
   * @brief Will first detect if the system is zero velocity, then will update.
   * @param state State of the filter
   * @param timestamp Next camera timestamp we want to see if we should propagate to.
   * @return True if the system is currently at zero velocity
   */
  bool try_update(std::shared_ptr<State> state, double timestamp);
protected:
    /// Container for the imu noise values
    NoiseManager _noises;

    /// Our propagator!
    std::shared_ptr<Propagator> _prop;
    /// Our history of IMU messages (time, angular, linear)
    std::vector<ov_core::ImuData> imu_data;

};

}// namespace ov_msckf

#endif