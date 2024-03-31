/*! @file PositionVelocityEstimator.h
 *  @brief All State Estimation Algorithms
 *
 *  This file will contain all state estimation algorithms.
 *  PositionVelocityEstimators should compute:
 *  - body position/velocity in world/body frames
 *  - foot positions/velocities in body/world frame
 */

#ifndef PROJECT_POSITIONVELOCITYESTIMATOR_H
#define PROJECT_POSITIONVELOCITYESTIMATOR_H

#include "stateEstData.h"
#include <iostream>

/*!
 * Position and velocity estimator based on a Kalman Filter.
 * This is the algorithm used in Mini Cheetah and Cheetah 3.
 */
namespace Est{
class LinearKFPositionVelocityEstimator {
  public:
    // EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    LinearKFPositionVelocityEstimator(StateEstimate &stateEst);
    
    void run(vec3d *hip2 , vec3d *ankleP2B, vec3d *ankleV2B, double *swPgs);
    void setup();
    vec3d *hip2_;
    vec3d *ankleP2B_;
    vec3d *ankleV2B_;

  private:
      StateEstimate &StateEstimateResult;
      Eigen::Matrix<double, 12, 1> _xhat;
      Eigen::Matrix<double, 6, 1> _ps;
      Eigen::Matrix<double, 6, 1> _vs;
      Eigen::Matrix<double, 12, 12> _A;
      Eigen::Matrix<double, 12, 12> _Q0;
      Eigen::Matrix<double, 12, 12> _P;
      Eigen::Matrix<double, 14, 14> _R0;
      Eigen::Matrix<double, 12, 3> _B;
      Eigen::Matrix<double, 14, 12> _C;
};
}

#endif  // PROJECT_POSITIONVELOCITYESTIMATOR_H
