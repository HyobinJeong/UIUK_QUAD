

/*! @file PositionVelocityEstimator.h
 *  @brief All State Estimation Algorithms
 *
 *  This file will contain all state estimation algorithms.
 *  PositionVelocityEstimators should compute:
 *  - body position/velocity in world/body frames
 *  - foot positions/velocities in body/world frame
 */

#ifndef POSITIONVELOCITYESTIMATOR
#define POSITIONVELOCITYESTIMATOR

#include <Eigen/Core>
#include <Eigen/Dense>
#include "cppTypes.h"
#include "orientation_tools.h"
#include "hubodog5_general.h"

/*!
 * Position and velocity estimator based on a Kalman Filter.
 * This is the algorithm used in Mini Cheetah and Cheetah 3.
 */

class PositionVelocityEstimator{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  PositionVelocityEstimator();
  void estimate(const Quat<double> &_quatBody,
                const Vec3<double> &_accBody,
                const Vec3<double> &_omegaBody,
                const Vec4<double> &_contactState,
                const RobotState &_quadState);
  void setup(const double _dt);

 public:
  Eigen::Matrix<double, 18, 1> _xhat;
  Eigen::Matrix<double, 12, 1> _ps;
  Eigen::Matrix<double, 12, 1> _vs;
  Eigen::Matrix<double, 18, 18> _A;
  Eigen::Matrix<double, 18, 18> _Q0;
  Eigen::Matrix<double, 18, 18> _P;
  Eigen::Matrix<double, 28, 28> _R0;
  Eigen::Matrix<double, 18, 3> _B;
  Eigen::Matrix<double, 28, 18> _C;

  Eigen::Matrix<double, 18, 18> At;
  Eigen::Matrix<double, 18, 18> Pm;
  Eigen::Matrix<double, 18, 28> Ct;
  Eigen::Matrix<double, 28, 1> yModel;
  Eigen::Matrix<double, 28, 1> ey;
  Eigen::Matrix<double, 28, 28> S;

public:
  Vec3<double> position;
  Vec3<double> vBody;
  Quat<double> orientation;
  Vec3<double> omegaBody;
  RotMat<double> rBody;
  Vec3<double> rpy;

  Vec3<double> omegaWorld;
  Vec3<double> vWorld;
  Vec3<double> aBody, aWorld;
};


#endif  // POSITIONVELOCITYESTIMATOR_H
