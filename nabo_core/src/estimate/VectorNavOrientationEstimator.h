//
// Created by billchent on 2020/9/23.
//
//
#ifndef SDUOG_CONTROLLER_ORIENTATIONESTIMATOR_H
#define SDUOG_CONTROLLER_ORIENTATIONESTIMATOR_H

#include "imu.h"
#include "stateEstData.h"
#include "../math/orientation_tools.h"
//#include <webots/types.h>
//#include <webots/robot.h>
//#include <webots/inertial_unit.h>
//#include <webots/gyro.h>
//#include <webots/gps.h>
//#include <webots/accelerometer.h>
//#include "../../../SDUogInclude.hpp"
// #include "StateEstimatorContainer.h"
// #include "state_estimator_lcmt.hpp"
// #include <lcm/lcm-cpp.hpp>
//#include "../config.h"
//using namespace webots;
/*!
 * "Cheater" estimator for orientation which always returns the correct value in simulation
 */
// template <typename T>
// class CheaterOrientationEstimator : public GenericEstimator<T>
// {
// public:
//     CheaterOrientationEstimator():lcm_pub_vec_nav("udpm://239.255.76.67:7667?ttl=1"){}
//     virtual void run();
//     virtual void setup() {}
//     lcm::LCM lcm_pub_vec_nav;
//     state_estimator_lcmt CheaterOrientationEstimator_vec_nav_res;
//     // lcm::LCM lcm_pub_vec_nav2;
//     // ("udpm://239.255.76.67:7667?ttl=1");
// };

/*!
 * Estimator for the VectorNav IMU.  The VectorNav provides an orientation already and
 * we just return that.
 */
namespace Est{
class VectorNavOrientationEstimator
{
public:
    VectorNavOrientationEstimator(StateEstimate &stateEst);
    void run(Rbt::imuClass &imu);
    // void setup() {}
    // void runtmp();
    // void Correct_Ori();
    // void Correct_Ori_dz();
    // void Correct_Ori_Ver2();
    // void Correct_Omega_Acc();
//    WbDeviceTag InerUnit;
//    WbDeviceTag Gyro;
//    WbDeviceTag Accelerometer;
//    WbDeviceTag Body_gps; // 辅助矫正,看估计的值和真实值的差距

private:
    StateEstimate &StateEstimateResult;
    bool _b_first_visit = true;
    Eigen::Matrix<double, 4, 1> _ori_ini_inv;
    // lcm::LCM lcm_pub_vec_nav;
    // state_estimator_lcmt CheaterOrientationEstimator_vec_nav_res;
    // state_estimator_lcmt CheaterOrientationEstimator_vec_nav_res_original;
};
}
#endif //SDUOG_CONTROLLER_ORIENTATIONESTIMATOR_H
