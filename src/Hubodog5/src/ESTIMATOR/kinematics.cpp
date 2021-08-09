//
// Created by legendarypyunny on 21. 1. 22..
//
#include "kinematics.h"

void Kinematics::getJacobian(int lnum,
                             const Eigen::Vector3d &joint_val,
                             Eigen::Matrix3d &J) {
  double R = joint_val[0];  // Roll angle
  double P = joint_val[1];  // Pitch angle
  double K = joint_val[2];  // Knee angle

  double l1 = hip2pitch * (2 * (lnum % 2) - 1); // -1 for the right and 1 for the left
  double l2 = thigh_length;
  double l3 = calf_length;

  J(0, 0) = 0;
  J(0, 1) = -l2*cos(P) - l3*cos(P+K);
  J(0, 2) = -l3*cos(P+K);

  J(1, 0) = -l1*sin(R) + cos(R)*(l2*cos(P) + l3*cos(P+K));
  J(1, 1) = -sin(R)*(l2*sin(P) + l3*sin(P+K));
  J(1, 2) = -l3*sin(R)*sin(P+K);

  J(2, 0) = l1*cos(R) + sin(R)*(l2*cos(P) + l3*cos(P+K));
  J(2, 1) = cos(R)*(l2*sin(P) + l3*sin(P+K));
  J(2, 2) = l3*cos(R)*sin(P+K);
}

void Kinematics::getJacobian(const Eigen::Matrix3d &rotation,
                             const Eigen::VectorXd &joint_val,
                             Eigen::MatrixXd &W_J) {
  W_J.setZero(12, 3);
  for (int i = 0; i < 4; i++) {
    Eigen::Matrix3d B_J;
    getJacobian(i, joint_val.segment(3*i, 3), B_J);
    W_J.middleRows(3*i, 3) = rotation * B_J;
  }
}

void Kinematics::getJacobianDerivative(int lnum,
                                  const Eigen::Vector3d& joint_pos,
                                  const Eigen::Vector3d& joint_vel,
                                  Eigen::Matrix3d& dJ) {
  int rightleft = 2*(lnum % 2) - 1; // -1 for right and 1 for left
  int fronthind = 2*(lnum > 1) - 1; // -1 for hind and 1 for front

  double q1 = joint_pos[0];  // Roll angle
  double q2 = joint_pos[1];  // Pitch angle
  double q3 = joint_pos[2];  // Knee angle

  double dq1 = joint_vel[0];  // Roll angle
  double dq2 = joint_vel[1];  // Pitch angle
  double dq3 = joint_vel[2];  // Knee angle

  double l1 = hip2pitch * rightleft; // -1 for the right and 1 for the left
  double l2 = - thigh_length;
  double l3 = - calf_length;

  dJ(0, 0) = 0;
  dJ(0, 1) = - dq2*(l3*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) + l2*sin(q2)) - dq3*l3*(cos(q2)*sin(q3) + cos(q3)*sin(q2));
  dJ(0, 2) = - dq2*l3*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) - dq3*l3*(cos(q2)*sin(q3) + cos(q3)*sin(q2));
  dJ(1, 0) = dq2*(l3*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)) + l2*cos(q1)*sin(q2)) -
                      dq1*(l3*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)) + l1*cos(q1) -
                      l2*cos(q2)*sin(q1)) + dq3*l3*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2));
  dJ(1, 1) = dq1*(l3*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)) + l2*cos(q1)*sin(q2)) -
                      dq2*(l3*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)) - l2*cos(q2)*sin(q1)) -
                      dq3*l3*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1));
  dJ(1, 2) = dq1*l3*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)) - dq2*l3*(sin(q1)*sin(q2)*sin(q3) -
                      cos(q2)*cos(q3)*sin(q1)) - dq3*l3*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1));
  dJ(2, 0) = dq2*(l3*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)) + l2*sin(q1)*sin(q2)) -
                      dq1*(l3*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)) + l1*sin(q1) +
                      l2*cos(q1)*cos(q2)) + dq3*l3*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2));
  dJ(2, 1) = dq1*(l3*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)) + l2*sin(q1)*sin(q2)) -
                      dq2*(l3*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)) + l2*cos(q1)*cos(q2)) -
                      dq3*l3*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3));
  dJ(2, 2) = dq1*l3*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)) - dq3*l3*(cos(q1)*cos(q2)*cos(q3) -
                      cos(q1)*sin(q2)*sin(q3)) - dq2*l3*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3));

}

Eigen::Vector3d Kinematics::getFootPosition(int lnum,
                                            const Eigen::Vector3d &joint_val) {
  double R = joint_val[0];  // Roll angle
  double P = joint_val[1];  // Pitch angle
  double K = joint_val[2];  // Knee angle

  int rightleft = 2*(lnum % 2) - 1; // -1 for right and 1 for left
  int fronthind = 2*(lnum > 1) - 1; // -1 for hind and 1 for front
  double l1 = hip2pitch * rightleft;
  double l2 = thigh_length;
  double l3 = calf_length;

  Eigen::Vector3d B_pos_hip2ft;
  B_pos_hip2ft[0] = -l2*sin(P) - l3*sin(P+K);
  B_pos_hip2ft[1] = l1*cos(R) + sin(R)*(l2*cos(P) + l3*cos(P+K));
  B_pos_hip2ft[2] = l1*sin(R) - cos(R)*(l2*cos(P) + l3*cos(P+K));

  Eigen::Vector3d B_pos_bd2hip;
  B_pos_bd2hip[0] = fronthind * bd2hip[0];
  B_pos_bd2hip[1] = rightleft * bd2hip[1];
  B_pos_bd2hip[2] = bd2hip[2];

  Eigen::Vector3d B_pos_bd2ft = B_pos_hip2ft + B_pos_bd2hip;
  return B_pos_bd2ft;
}

Eigen::Vector3d Kinematics::getFootVelocity(int lnum,
                                            const Eigen::Vector3d& joint_val,
                                            const Eigen::Vector3d& joint_speed) {
  Eigen::Matrix3d J;
  getJacobian(lnum, joint_val, J);

  Eigen::Vector3d B_vel_bd2ft = J * joint_speed;
  return B_vel_bd2ft;
}

Eigen::Vector3d Kinematics::getFootPosition(int lnum,
                                            const Eigen::Matrix3d &rotation,
                                            const Eigen::Vector3d &joint_val) {
  Eigen::Vector3d W_pos_bd2ft = rotation * getFootPosition(lnum, joint_val);
  Eigen::Vector3d W_pos_ft2gr = Eigen::Vector3d(0, 0, -foot_radius);
  return W_pos_bd2ft + W_pos_ft2gr;
}

void Kinematics::getFootPosition(const Eigen::Vector3d &body_pos,
                                 const Eigen::Matrix3d &rotation,
                                 const Eigen::VectorXd &joint_val,
                                 Eigen::MatrixXd& foot_position) {
  foot_position.setZero(3, 4);
  for(int i = 0; i < 4; i++)
    foot_position.col(i) = body_pos + getFootPosition(i, rotation,
                                                      joint_val.segment(3*i, 3));
}

Eigen::Vector3d Kinematics::getFootVelocity(int lnum,
                                            const Eigen::Matrix3d &rotation,
                                            const Eigen::Vector3d &W_ang_vel,
                                            const Eigen::Vector3d &joint_val,
                                            const Eigen::Vector3d &joint_speed) {
  double R = joint_val[0];
  double Rdot = joint_speed(0);
  double Pdot = joint_speed(1);
  double Kdot = joint_speed(2);

  Eigen::Vector3d B_ang_vel_ft = Eigen::Vector3d(Rdot, cos(R)*(Kdot+Pdot), sin(R)*(Kdot+Pdot));
  Eigen::Vector3d W_ang_vel_ft = W_ang_vel + rotation * B_ang_vel_ft;

  Eigen::Vector3d W_pos_ft2gr = Eigen::Vector3d(0, 0, -foot_radius);
  Eigen::Vector3d W_pos_bd2ft = rotation * getFootPosition(lnum, joint_val) - W_pos_ft2gr;
  Eigen::Vector3d W_vel_bd2ft = W_ang_vel.cross(W_pos_bd2ft) + rotation * getFootVelocity(lnum, joint_val, joint_speed);

  Eigen::Vector3d W_vel_ft2gr = W_ang_vel_ft.cross(W_pos_ft2gr);
  return W_vel_bd2ft + W_vel_ft2gr;
}

void Kinematics::getFootVelocity(const Eigen::Vector3d &body_vel,
                                 const Eigen::Matrix3d &rotation,
                                 const Eigen::Vector3d &W_ang_vel,
                                 const Eigen::VectorXd &joint_val,
                                 const Eigen::VectorXd &joint_speed,
                                 Eigen::MatrixXd &foot_velocity) {
  foot_velocity.setZero(3, 4);
  for(int i = 0; i < 4; i++)
    foot_velocity.col(i) = body_vel + getFootVelocity(i, rotation, W_ang_vel,
                                                      joint_val.segment(3*i, 3),
                                                      joint_speed.segment(3*i, 3));
}

Eigen::Vector3d Kinematics::getIMU2FootPosition(int lnum,
                                                const Eigen::Matrix3d &rotation,
                                                const Eigen::Vector3d &joint_val) {
  Eigen::Vector3d W_pos_bd2gr = getFootPosition(lnum, rotation, joint_val);
  Eigen::Vector3d B_pos_imu2bd;
  B_pos_imu2bd << -bd2imu[0], -bd2imu[1], -bd2imu[2];
  Eigen::Vector3d W_pos_imu2bd = rotation * B_pos_imu2bd;

  Eigen::Vector3d W_pos_imu2gr = W_pos_imu2bd + W_pos_bd2gr;
  return W_pos_imu2gr;
}

Eigen::Vector3d Kinematics::getIMU2FootVelocity(int lnum,
                                                const Eigen::Matrix3d &rotation,
                                                const Eigen::Vector3d &W_ang_vel,
                                                const Eigen::Vector3d &joint_val,
                                                const Eigen::Vector3d &joint_vel) {
  Eigen::Matrix3d J;
  getJacobian(lnum, joint_val, J);

  Eigen::Vector3d b_vel_imu2ft = J * joint_vel;

  double R = joint_val(0);
  double Rdot = joint_vel(0);
  double Pdot = joint_vel(1);
  double Kdot = joint_vel(2);

  Eigen::Vector3d b_ang_vel_ft(Rdot, cos(R)*(Kdot+Pdot), sin(R)*(Kdot+Pdot));
  Eigen::Vector3d W_ang_vel_ft = W_ang_vel + rotation * b_ang_vel_ft;

  Eigen::Vector3d W_pos_ft2gr(0, 0, -foot_radius);
  Eigen::Vector3d W_vel_ft2gr = W_ang_vel_ft.cross(W_pos_ft2gr);

  Eigen::Vector3d W_pos_imu2ft = getIMU2FootPosition(lnum, rotation, joint_val) - W_pos_ft2gr;

  Eigen::Vector3d W_vel_imu2gr = W_ang_vel.cross(W_pos_imu2ft) + rotation * b_vel_imu2ft + W_vel_ft2gr;
  return W_vel_imu2gr;
}

void Kinematics::imu2Bd(const Eigen::Matrix3d &rotation,
                        const Eigen::Vector3d &W_ang_vel,
                        const Eigen::Vector3d &W_pos_imu,
                        const Eigen::Vector3d &W_vel_imu,
                        Eigen::Vector3d &W_pos_bd,
                        Eigen::Vector3d &W_vel_bd) {
  Eigen::Vector3d B_pos_imu2bd;
  B_pos_imu2bd << -bd2imu[0], -bd2imu[1], -bd2imu[2];
  Eigen::Vector3d W_pos_imu2bd = rotation * B_pos_imu2bd;
  W_pos_bd = W_pos_imu + W_pos_imu2bd;
  W_vel_bd = W_vel_imu + W_ang_vel.cross(W_pos_imu2bd);
}



