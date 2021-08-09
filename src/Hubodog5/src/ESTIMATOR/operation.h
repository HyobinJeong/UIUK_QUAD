//
// Created by legendarypyunny on 2020-07-28.
//

#ifndef _ADD_GNNMPC_OPERATION_H_
#define _ADD_GNNMPC_OPERATION_H_

#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <random>
#include <chrono>
#include <iostream>
#include <fstream>

//#include "casadi/nonlinearDynamics.h"
//#include "casadi/dfdx.h"
//#include "casadi/dfdu.h"
//#include "casadi/next_bezier.h"
//#include "casadi/quadrupedJacobian.h"
//#include "casadi/forwardKinematics.h"

//#include "qpswift/Prime.h"

#include "robot_parameters.h"


class Operation {
 public:
  explicit Operation(const double dt);

  // Quaternion manipulations
  static void quatToRotation(const Eigen::Vector4d& quaternion, Eigen::Matrix3d& rotation);
  static Eigen::Vector3d quatToEul(const Eigen::Vector4d& quat);
  static Eigen::Vector4d quatProduct(const Eigen::Vector4d &a,
                                     const Eigen::Vector4d &b);

  // Matrix manipulations
  static void rotationIntegration(Eigen::Matrix3d& rotation, double dt,
      const Eigen::Vector3d& angular_velocity);
  static Eigen::Vector3d matrixLog(const Eigen::Matrix3d &R);


  // Sparse Manipulation
  static void vstack(
      Eigen::SparseMatrix<double> &A,
      const Eigen::SparseMatrix<double> &B);

  static Eigen::VectorXd NormalRandom (int n);



  // Bezier Curve
  static void getBezierPoints(const Eigen::MatrixXd& points,
                              const Eigen::VectorXd& times,
                              Eigen::MatrixXd& position,
                              Eigen::MatrixXd& velocity,
                              Eigen::MatrixXd& acc);

  static void drawBezier(const Eigen::Vector3d &start,
                         const Eigen::Vector3d &middle,
                         const Eigen::Vector3d &final,
                         const Eigen::Vector3d &mid_vel,
                         double duration,
                         Eigen::MatrixXd &pos,
                         Eigen::MatrixXd &vel,
                         Eigen::MatrixXd &acc);

  static void getSwingRef(double duration,
                          double elapsed,
                          const Eigen::Vector3d &start,
                          const Eigen::Vector3d &middle,
                          const Eigen::Vector3d &final,
                          Eigen::Vector3d& p_target,
                          Eigen::Vector3d& d_target,
                          Eigen::Vector3d& a_target);

  // QP Swift
//  static bool solveQP(
//      const Eigen::SparseMatrix<double> &P,
//      const Eigen::SparseMatrix<double> &A,
//      const Eigen::SparseMatrix<double> &G,
//      const Eigen::VectorXd &q,
//      const Eigen::VectorXd &b,
//      const Eigen::VectorXd &h,
//      double &qp_time,
//      qp_int **permut,
//      Eigen::VectorXd &solution);

 private:
  const double dt_;

  static inline double factorial(int num) {
    if(num == 0)
      return 1;
    else
      return num * factorial(num - 1);
  }
};


#endif //_ADD_GNNMPC_OPERATION_H_
