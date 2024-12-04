#include "UpdaterGNSS.h"
#include "state/Propagator.h"
#include "state/State.h"
#include "state/StateHelper.h"
#include "utils/pos_transform.h"

using namespace ov_type;
using namespace ov_msckf;

UpdaterGNSS::UpdaterGNSS(std::shared_ptr<Propagator> prop): _prop(prop)
{

}

bool UpdaterGNSS::try_update(std::shared_ptr<State> state, const ov_core::GnssData &gnss_data,const Eigen::Vector3d &init_lla,
                const Eigen::Matrix3d &init_R_GNSStoI,const Eigen::Vector3d &init_t_GNSStoI)
{
    // Return if the gnss data timestamp before state 
    if(state->_timestamp - gnss_data.timestamp > 1e-6) {
        PRINT_WARNING(YELLOW "gnss data received out of order, unable to do anything (prop dt = %3f)\n" RESET,
                  (gnss_data.timestamp - state->_timestamp));
        return false;
    }

    // Return if the gnss data is not reliable
    if(gnss_data.status_pos == 0){
        return false;
    }

    std::vector<double> lla0(3),lla1(3);
    lla0[0]=init_lla[0],lla0[1]=init_lla[1],lla0[2]=init_lla[2];
    lla1[0]=gnss_data.latitude,lla1[1]=gnss_data.longitude,lla1[2]=gnss_data.altitude;
    std::vector<double> enu = lla2enu(lla1,lla0);
    Eigen::Vector3d pos_in_gnss,pos_in_imu,std_in_gnss,std_in_imu;
    pos_in_gnss[0] = enu[0],pos_in_gnss[1] = enu[1],pos_in_gnss[2] = enu[2];
    pos_in_imu = init_R_GNSStoI * pos_in_gnss + init_t_GNSStoI;
    std_in_gnss[0] = gnss_data.std_latitude,std_in_gnss[1] = gnss_data.std_longitude,std_in_gnss[2] = gnss_data.std_altitude;
    std_in_imu = init_R_GNSStoI * std_in_gnss;

    Eigen::Vector3d state_pos = state->_imu->pos();
    PRINT_INFO("gnss timestamp: %.2f,gnss pos:(%.2f,%.2f,%.2f)\n", gnss_data.timestamp,pos_in_imu[0],pos_in_imu[1],pos_in_imu[2]);
    PRINT_INFO("state timestamp: %.2f,gnss pos:(%.2f,%.2f,%.2f)\n", state->_timestamp,state_pos[0],state_pos[1],state_pos[2]);
    _prop->propagate_and_clone(state, gnss_data.timestamp);
    state_pos = state->_imu->pos();

    // Create the update system!
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, 3);
    Eigen::VectorXd res = Eigen::VectorXd::Zero(3);
    Eigen::MatrixXd R = Eigen::MatrixXd::Zero(3, 3);
    std::vector<std::shared_ptr<Type>> Hx_order;
    res.block(0, 0, 3, 1) = pos_in_imu - state_pos;

    Hx_order.push_back(state->_imu->p());

    H.block(0, 0, 3, 3) = Eigen::Matrix3d::Identity();
    // noise (order is ori, pos, vel)
    R(0,0) = std_in_imu[0]*std_in_imu[0];
    R(1,1) = std_in_imu[1]*std_in_imu[1];
    R(2,2) = std_in_imu[2]*std_in_imu[2];

    StateHelper::EKFUpdate(state, Hx_order, H, res, R);
    StateHelper::marginalize(state, state->_clones_IMU.at(gnss_data.timestamp));
    state->_clones_IMU.erase(gnss_data.timestamp);
    state_pos = state->_imu->pos();
    PRINT_INFO("fuse state timestamp: %.2f,fuse state pos:(%.2f,%.2f,%.2f)\n", state->_timestamp,state_pos[0],state_pos[1],state_pos[2]);
    return true;
}