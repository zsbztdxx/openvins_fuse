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

bool UpdaterGNSS::try_update(std::shared_ptr<State> state, const ov_core::SatNavData &gnss_data, const Eigen::Vector3d &init_lla,
                const Eigen::Matrix3d &init_R_GNSStoI, const Eigen::Vector3d &init_t_GNSStoI) {
    
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
    PRINT_INFO("gnss timestamp: %.5f,gnss pos:(%.2f,%.2f,%.2f)\n", gnss_data.timestamp,pos_in_imu[0],pos_in_imu[1],pos_in_imu[2]);
    PRINT_INFO("state timestamp: %.5f,state pos:(%.2f,%.2f,%.2f)\n", state->_timestamp,state_pos[0],state_pos[1],state_pos[2]);
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
    PRINT_INFO("fuse state timestamp: %.5f,fuse state pos:(%.2f,%.2f,%.2f)\n", state->_timestamp,state_pos[0],state_pos[1],state_pos[2]);
    return true;
}

bool UpdaterGNSS::try_update(std::shared_ptr<State> state, const ov_core::InsPvaData &ins_gnss_data, const Eigen::Vector3d &init_lla,
                const Eigen::Matrix3d &init_R_GNSStoI, const Eigen::Vector3d &init_t_GNSStoI) {
    
    // Return if the gnss data timestamp before state 
    if(state->_timestamp > ins_gnss_data.timestamp) {
        PRINT_WARNING(YELLOW "gnss data received out of order, unable to do anything (prop dt = %3f)\n" RESET,
                  (ins_gnss_data.timestamp - state->_timestamp));
        return false;
    }

    // Propagate the state forward to the current update time
    // Also augment it with a new clone!
    // NOTE: if the state is already at the given time (can happen in sim)
    // NOTE: then no need to prop since we already are at the desired timestep
    if (state->_timestamp == ins_gnss_data.timestamp) {
        PRINT_WARNING(YELLOW "the state is already at the given time, will return soon!\n" RESET);
        // state->_clones_IMU.erase(state->_timestamp);
        return false;
    }

    std::vector<double> lla0(3), lla1(3);
    lla0[0]=init_lla[0], lla0[1]=init_lla[1], lla0[2]=init_lla[2];
    lla1[0]=ins_gnss_data.ins_lat, lla1[1]=ins_gnss_data.ins_lon, lla1[2]=ins_gnss_data.ins_hgt;
    std::vector<double> enu = lla2enu(lla1,lla0);
    Eigen::Vector3d pos_in_gnss, pos_in_imu, std_in_gnss, std_in_imu;
    pos_in_gnss[0] = enu[0], pos_in_gnss[1] = enu[1], pos_in_gnss[2] = enu[2];
    pos_in_imu = init_R_GNSStoI * pos_in_gnss + init_t_GNSStoI;

    // std_in_gnss[0] = gnss_data.std_latitude,std_in_gnss[1] = gnss_data.std_longitude,std_in_gnss[2] = gnss_data.std_altitude;
    // std_in_imu = init_R_GNSStoI * std_in_gnss;

    Eigen::Vector3d state_pos = state->_imu->pos();
    PRINT_INFO("gnss timestamp: %.2f, gnss pos:(%.2f,%.2f,%.2f)\n", ins_gnss_data.timestamp,pos_in_imu[0],pos_in_imu[1],pos_in_imu[2]);
    PRINT_INFO("state timestamp: %.2f, state pos:(%.2f,%.2f,%.2f)\n", state->_timestamp,state_pos[0],state_pos[1],state_pos[2]);
    
    _prop->propagate_and_clone(state, ins_gnss_data.timestamp);
    state_pos = state->_imu->pos();

    // Create the update system!
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, 3);
    Eigen::VectorXd res = Eigen::VectorXd::Zero(3);
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(3, 3);
    std::vector<std::shared_ptr<Type>> Hx_order;
    res.block(0, 0, 3, 1) = pos_in_imu - state_pos;

    Hx_order.push_back(state->_imu->p());

    H.block(0, 0, 3, 3) = Eigen::Matrix3d::Identity();
    // noise
    for (int i = 0; i < R.rows(); ++i) {
        R(i, i) *= 0.01;
    }

    StateHelper::EKFUpdate(state, Hx_order, H, res, R);
    StateHelper::marginalize(state, state->_clones_IMU.at(ins_gnss_data.timestamp));
    state->_clones_IMU.erase(ins_gnss_data.timestamp);
    state_pos = state->_imu->pos();
    PRINT_INFO("fuse state timestamp: %.5f,fuse state pos:(%.2f,%.2f,%.2f)\n", state->_timestamp,state_pos[0],state_pos[1],state_pos[2]);
    return true;
}

bool UpdaterGNSS::try_update(std::shared_ptr<State> state, const ov_core::InsPvaData &ins_gnss_data, const int &rover_fix_level,
                const Eigen::Vector3d &init_lla, const Eigen::Matrix3d &init_R_GNSStoI, const Eigen::Vector3d &init_t_GNSStoI) {
    
    // Return if the gnss data timestamp before state 
    if(state->_timestamp > ins_gnss_data.timestamp) {
        PRINT_WARNING(YELLOW "gnss data received out of order, unable to do anything (prop dt = %3f)\n" RESET,
                  (ins_gnss_data.timestamp - state->_timestamp));
        return false;
    }

    // Propagate the state forward to the current update time
    // Also augment it with a new clone!
    // NOTE: if the state is already at the given time (can happen in sim)
    // NOTE: then no need to prop since we already are at the desired timestep
    if (state->_timestamp == ins_gnss_data.timestamp) {
        PRINT_WARNING(YELLOW "the state is already at the given time, will return soon!\n" RESET);
        // state->_clones_IMU.erase(state->_timestamp);
        return false;
    }

    std::vector<double> lla0(3), lla1(3);
    lla0[0]=init_lla[0], lla0[1]=init_lla[1], lla0[2]=init_lla[2];
    lla1[0]=ins_gnss_data.ins_lat, lla1[1]=ins_gnss_data.ins_lon, lla1[2]=ins_gnss_data.ins_hgt;
    std::vector<double> enu = lla2enu(lla1,lla0);
    Eigen::Vector3d pos_in_gnss, pos_in_imu, std_in_gnss, std_in_imu;
    pos_in_gnss[0] = enu[0], pos_in_gnss[1] = enu[1], pos_in_gnss[2] = enu[2];
    pos_in_imu = init_R_GNSStoI * pos_in_gnss + init_t_GNSStoI;

    // std_in_gnss[0] = gnss_data.std_latitude,std_in_gnss[1] = gnss_data.std_longitude,std_in_gnss[2] = gnss_data.std_altitude;
    // std_in_imu = init_R_GNSStoI * std_in_gnss;

    Eigen::Vector3d state_pos = state->_imu->pos();
    PRINT_INFO("gnss timestamp: %.2f, gnss pos:(%.2f,%.2f,%.2f)\n", ins_gnss_data.timestamp,pos_in_imu[0],pos_in_imu[1],pos_in_imu[2]);
    PRINT_INFO("state timestamp: %.2f, state pos:(%.2f,%.2f,%.2f)\n", state->_timestamp,state_pos[0],state_pos[1],state_pos[2]);
    
    _prop->propagate_and_clone(state, ins_gnss_data.timestamp);
    state_pos = state->_imu->pos();

    // Create the update system!
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, 3);
    Eigen::VectorXd res = Eigen::VectorXd::Zero(3);
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(3, 3);
    std::vector<std::shared_ptr<Type>> Hx_order;
    res.block(0, 0, 3, 1) = pos_in_imu - state_pos;

    Hx_order.push_back(state->_imu->p());

    H.block(0, 0, 3, 3) = Eigen::Matrix3d::Identity();

    // different weight of filter(especially noise, and K)
    // mainly depend on hdop and rover_fix_level, sat_num hasn't been used yet 
    if (ins_gnss_data.hdop < 2) {
        // noise
        for (int i = 0; i < R.rows(); ++i) {
            R(i, i) *= 1e-10;
        }

        // // 打印更新阶段gnss测量噪声上三角矩阵视图
        // Eigen::MatrixXd Rg = R.selfadjointView<Eigen::Upper>();
        // PRINT_INFO(YELLOW "更新阶段gnss测量噪声上三角矩阵的值:\n", RESET);
        // for (int i = 0; i < Rg.rows(); ++i) {
        //     for (int j = i; j < Rg.cols(); ++j) {
        //         std::cout << Rg(i, j);
        //         if (j < Rg.cols() - 1) {
        //             std::cout << " ";
        //         }
        //     }
        //     std::cout << std::endl;
        // }

    } else if (2 <= ins_gnss_data.hdop && ins_gnss_data.hdop < 5) {
        // noise
        for (int i = 0; i < R.rows(); ++i) {
            R(i, i) *= 1e-8;
        }
    } else {
        // noise
        for (int i = 0; i < R.rows(); ++i) {
            R(i, i) *= 1e-3;
        }
    } 

    StateHelper::GNSS_EKFUpdate(state, Hx_order, H, res, R, ins_gnss_data.hdop, rover_fix_level, ins_gnss_data.gnss_status);
    // StateHelper::EKFUpdate(state, Hx_order, H, res, R);
    // PRINT_INFO("_clone_IMU中ins_gnss_data.timestamp时刻的state数量为: %d\n", state->_clones_IMU.count(ins_gnss_data.timestamp));
    StateHelper::marginalize(state, state->_clones_IMU.at(ins_gnss_data.timestamp));

    state->_clones_IMU.erase(ins_gnss_data.timestamp);
    state_pos = state->_imu->pos();
    PRINT_INFO("hdop: %.2f | fuse state timestamp: %.2f, fuse state pos:(%.2f,%.2f,%.2f)\n", ins_gnss_data.hdop,state->_timestamp,state_pos[0],state_pos[1],state_pos[2]);
    return true;   
}