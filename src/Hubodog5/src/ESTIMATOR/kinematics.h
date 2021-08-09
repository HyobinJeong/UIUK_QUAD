//
// Created by legendarypyunny on 21. 1. 22..
//

#ifndef KINEMATCIS_H
#define KINEMATCIS_H

#include <Eigen/Core>
#include <Eigen/Dense>

#include "robot_parameters.h"

class Kinematics {
public:
    static void getJacobian(const Eigen::Matrix3d& rotation,
                            const Eigen::VectorXd& joint_val,
                            Eigen::MatrixXd& W_J);

    static Eigen::Vector3d getFootPosition(int lnum,
                                           const Eigen::Matrix3d& rotation,
                                           const Eigen::Vector3d& joint_val);
    static Eigen::Vector3d getFootVelocity(int lnum,
                                           const Eigen::Matrix3d& rotation,
                                           const Eigen::Vector3d& W_ang_vel,
                                           const Eigen::Vector3d& joint_val,
                                           const Eigen::Vector3d& joint_speed);

    static void getFootPosition(const Eigen::Vector3d& body_pos,
                                const Eigen::Matrix3d& rotation,
                                const Eigen::VectorXd& joint_val,
                                Eigen::MatrixXd& foot_position);

    static void getFootVelocity(const Eigen::Vector3d& body_vel,
                                const Eigen::Matrix3d& rotation,
                                const Eigen::Vector3d& W_ang_vel,
                                const Eigen::VectorXd& joint_val,
                                const Eigen::VectorXd& joint_speed,
                                Eigen::MatrixXd& foot_velocity);

    static Eigen::Vector3d getIMU2FootPosition(int lnum,
                                               const Eigen::Matrix3d &rotation,
                                               const Eigen::Vector3d &joint_val);

    static Eigen::Vector3d getIMU2FootVelocity(int lnum,
                                               const Eigen::Matrix3d &rotation,
                                               const Eigen::Vector3d &W_ang_vel,
                                               const Eigen::Vector3d &joint_val,
                                               const Eigen::Vector3d &joint_vel);

    static void imu2Bd(const Eigen::Matrix3d &rotation,
                       const Eigen::Vector3d &W_ang_vel,
                       const Eigen::Vector3d &W_pos_imu,
                       const Eigen::Vector3d &W_vel_imu,
                       Eigen::Vector3d &W_pos_bd,
                       Eigen::Vector3d &W_vel_bd);

    static void getJacobian(int lnum,
                            const Eigen::Vector3d& joint_val,
                            Eigen::Matrix3d& J);

    static void getJacobianDerivative(int lnum,
                                      const Eigen::Vector3d& joint_pos,
                                      const Eigen::Vector3d& joint_vel,
                                      Eigen::Matrix3d& dJ);

    static Eigen::Vector3d getFootPosition(int lnum,
                                           const Eigen::Vector3d& joint_val);
    static Eigen::Vector3d getFootVelocity(int lnum,
                                           const Eigen::Vector3d& joint_val,
                                           const Eigen::Vector3d& joint_speed);

};  // class Kinematics

#endif
