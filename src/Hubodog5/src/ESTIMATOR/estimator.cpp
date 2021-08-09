#include "estimator.h"

Estimator::Estimator(const Eigen::Vector3d &init_pos,
                     const Eigen::VectorXd &init_joint,
                     const Eigen::Vector4d &init_quat,
                     double dt) : quat_(init_quat), dt_(dt) {
  xhat_.setZero(18);
  Eigen::Matrix3d init_R;
  Operation::quatToRotation(init_quat, init_R);

  Eigen::Vector3d B_bd2imu(bd2imu[0], bd2imu[1], bd2imu[2]);
  xhat_.head(3) = init_pos + init_R.transpose() * B_bd2imu;

  for (int i = 0; i < 4; i++) {
    xhat_.segment(6 + 3*i, 3) = init_pos + Kinematics::getFootPosition(i, init_R, init_joint.segment(3*i, 3));
    xhat_(8 + 3*i) = 0;
  }

  //for Mahoney AHRS error integral term init
  eInt_.setZero();

  //for contact estimation
  residual_prev_.setZero();
  momentum_hat_.setZero();
  contact_Ki_ << 280, 200, 250, 220;
  contact_Ki_.setConstant(150);
  calf_I_ = calf_Iyy + calf_mass * calf_com[2] * calf_com[2];
  contact_threshold_ = 2.5;

  double imu_pos_cov = 1;  /// Suspicious (Too much drift by double integral)
  double imu_vel_cov = imu_pos_cov;  /// Reasonable but still exists drift
  double ft_pos_cov = 10;   /// Can't believe at all

  int imu_enc_ratio = 10;
  double enc_pos_cov = imu_pos_cov/imu_enc_ratio;  /// Accurate theoretically(trustworthy)
  double enc_vel_cov = enc_pos_cov*5;  /// Accurate only when contact is firm + likely to be noisy
  double ft_height_cov = enc_pos_cov*2.5;/// Accurate only when contact is firm

  setCovariance(imu_pos_cov, imu_vel_cov, ft_pos_cov, enc_pos_cov, enc_vel_cov, ft_height_cov);

  quat_log.open("quat.txt");
  est_log.open("est.txt");
  contact_log.open("contact.txt");
}

Estimator::Estimator(double dt) :dt_(dt) {
  Eigen::Vector3d init_pos(0, 0, height);
  Eigen::Vector4d init_quat(1, 0, 0, 0);
  quat_ = init_quat;

  Eigen::VectorXd init_joint(12);
  for(int i = 0; i < 4; i++)
    init_joint.segment(3*i, 3) << 0, init_angle, -2 * init_angle;

  xhat_.setZero(18);
  Eigen::Vector3d B_bd2imu(bd2imu[0], bd2imu[1], bd2imu[2]);
  xhat_.head(3) = init_pos + B_bd2imu;

  for (int i = 0; i < 4; i++) {
    xhat_.segment(6 + 3*i, 3) = init_pos + Kinematics::getFootPosition(i, Eigen::Matrix3d::Identity(), init_joint.segment(3*i, 3));
    xhat_(8 + 3*i) = 0;
  }

  eInt_.setZero();

  residual_prev_.setZero();
  momentum_hat_.setZero();
  contact_Ki_ << 280, 200, 250, 220;
  contact_Ki_.setConstant(150);
  calf_I_ = calf_Iyy + calf_mass * calf_com[2] * calf_com[2];
  contact_threshold_ = 2.5;

  double imu_pos_cov = 1;  /// Suspicious (Too much drift by double integral)
  double imu_vel_cov = imu_pos_cov*10;  /// Reasonable but still exists drift
  double ft_pos_cov = 10;   /// Can't believe at all

  double imu_enc_ratio = 1000;
  double enc_pos_cov = imu_pos_cov/imu_enc_ratio;  /// Accurate theoretically(trustworthy)
  double enc_vel_cov = enc_pos_cov*500000;  /// Accurate only when contact is firm + likely to be noisy
  double ft_height_cov = enc_pos_cov*250000;/// Accurate only when contact is firm

  setCovariance(imu_pos_cov, imu_vel_cov, ft_pos_cov, enc_pos_cov, enc_vel_cov, ft_height_cov);

  quat_log.open("quat.txt");
  est_log.open("est.txt");
  contact_log.open("contact.txt");
}

void Estimator::Init(const Eigen::Vector3d &init_pos,
                     const Eigen::VectorXd &init_joint,
                     const Eigen::Vector4d &init_quat){
    xhat_.setZero(18);
    Eigen::Matrix3d init_R;
    Operation::quatToRotation(init_quat, init_R);

    Eigen::Vector3d B_bd2imu(bd2imu[0], bd2imu[1], bd2imu[2]);
    xhat_.head(3) = init_pos + init_R.transpose() * B_bd2imu;

    for (int i = 0; i < 4; i++) {
      xhat_.segment(6 + 3*i, 3) = init_pos + Kinematics::getFootPosition(i, init_R, init_joint.segment(3*i, 3));
      xhat_(8 + 3*i) = 0;
    }

}


Estimator::~Estimator() {
  quat_log.close();
  est_log.close();
  contact_log.close();
}

void Estimator::setCovariance(double imu_pos_cov,
                              double imu_vel_cov,
                              double ft_pos_cov,
                              double enc_pos_cov,
                              double enc_vel_cov,
                              double ft_height_cov) {

  A_.setZero(18, 18);
  A_.block(0, 0, 3, 3).setIdentity();
  A_.block(0, 3, 3, 3) = dt_ * Eigen::Matrix3d::Identity();
  A_.block(3, 3, 3, 3).setIdentity();
  A_.block(6, 6, 12, 12).setIdentity();

  B_.setZero(18, 3);
  B_.middleRows(3, 3) = dt_ * Eigen::Matrix3d::Identity();

  C_.setZero(28, 18);
  for(int i = 0; i < 4; i++) {
    C_.block(3*i, 0, 3, 3).setIdentity();
    C_.block(12 + 3*i, 3, 3, 3).setIdentity();
    C_(24 + i, 8 + 3*i) = 1;
  }
  C_.block(0, 6, 12, 12) = -1 * Eigen::Matrix<double, 12, 12>::Identity();

  P_.setIdentity(18, 18);
  P_ *= 100;

  Eigen::VectorXd q(18);
  for(int i = 0; i < 3; i++) {
    q(i) = imu_pos_cov;
    q(3 + i) = imu_vel_cov;
    for(int j = 0; j < 4; j++)
      q(6 + 4*i + j) = ft_pos_cov;
  }
  Q_ = q.asDiagonal();

  Eigen::VectorXd r(28);
  for(int i = 0; i < 4; i++) {
    for(int j = 0; j < 3; j++) {
      r(3*i + j) = enc_pos_cov;
      r(12 + 3*i + j) = enc_vel_cov;
    }
    r(24 + i) = ft_height_cov;
  }
  R_ = r.asDiagonal();
}

void Estimator::complementaryFilter(const Eigen::Vector3d &gyro,
                                    const Eigen::Vector3d &acc) {
  if(acc.norm() < 1e-12)
    return;

  quat_log << acc(0) << ", " << acc(1) << ", " << acc(2) << ", ";
  quat_log << gyro(0) << ", " << gyro(1) << ", " << gyro(2) << ", ";

  // Estimated direction of gravity;
  Eigen::Vector3d v;
  v << 2 * (quat_[1]*quat_(3)-quat_(0)*quat_(2)),
      2 * (quat_[0]*quat_[1]+quat_[2]*quat_[3]),
      quat_[0]*quat_[0]-quat_[1]*quat_[1]-quat_[2]*quat_[2]+quat_[3]*quat_[3];
  quat_log << v(0) << ", " << v(1) << ", " << v(2) << ", ";

  Eigen::Vector3d e = acc.normalized().cross(v);
  eInt_ += e * dt_;
  Eigen::Vector3d gyro_modified = gyro + imu_Kp * e + imu_Ki * eInt_;

  Eigen::Vector4d q_dot =
          0.5 * Operation::quatProduct(quat_, Eigen::Vector4d(0, gyro_modified(0), gyro_modified(1), gyro_modified(2)));
  quat_log << q_dot(0) << ", " << q_dot(1) << ", " << q_dot(2) << ", " << q_dot(3) << ", ";

  quat_ += q_dot * dt_;
  quat_ = quat_.normalized();
  quat_log << quat_(0) << ", " << quat_(1) << ", " << quat_(2) << ", " << quat_(3) << "\n";

}

void Estimator::reset_complementaryFilter(const Eigen::Vector4d &_init_quat){
    eInt_.setZero();
    quat_ = _init_quat.normalized();
}

void Estimator::detectContact(const Eigen::Vector4d &knee_vel,
                              const Eigen::Vector4d &knee_input,
                              Eigen::Vector4i &contact) {

  Eigen::Vector4d momentum = calf_I_ * knee_vel;
  Eigen::Vector4d duk = knee_input - residual_prev_;
  momentum_hat_ += duk * dt_;

  Eigen::Vector4d residual = contact_Ki_.array() * (momentum_hat_ - momentum).array();
  residual_prev_ = residual;
  contact_log << knee_vel(0) << ", " << knee_vel(1) << ", " << knee_vel(2) << ", " << knee_vel(3) << ", ";
  contact_log << knee_input(0) << ", " << knee_input(1) << ", " << knee_input(2) << ", " << knee_input(3) << ", ";

  for (int i = 0; i < 4; i++)
    if(residual[i] > contact_threshold_)
      contact[i] = 1;
    else
      contact[i] = 0;
  contact_log << contact(0) << ", " << contact(1) << ", " << contact(2) << ", " << contact(3) << "\n";

}

void Estimator::estimate(const Eigen::Matrix3d &rotation,
                         const Eigen::Vector3d &W_acc_imu,
                         const Eigen::Vector3d &W_ang_vel,
                         const Eigen::Vector4i& contact,
                         const Eigen::VectorXd &joint_pos,
                         const Eigen::VectorXd &joint_vel,
                         Eigen::Vector3d &W_pos_bd,
                         Eigen::Vector3d &W_vel_bd) {  // W means 'World'
  Eigen::Vector3d p0 = xhat_.head(3);
  Eigen::Vector3d v0 = xhat_.segment(3, 3);

  Eigen::VectorXd W_pos_fts2imu(12);
  Eigen::VectorXd W_vel_fts2imu(12);
  Eigen::Vector4d W_height_ft;
  W_height_ft.setZero();

  Eigen::Vector3d W_pos_imu2ft, W_vel_imu2ft;
  for(int i = 0; i < 4; i++) {
    W_pos_imu2ft = Kinematics::getIMU2FootPosition(i, rotation,
                                                   joint_pos.segment(3*i, 3));
    W_vel_imu2ft = Kinematics::getIMU2FootVelocity(i, rotation,
                                                   W_ang_vel,
                                                   joint_pos.segment(3*i, 3),
                                                   joint_vel.segment(3*i, 3));

    W_pos_fts2imu.segment(3 * i, 3) = -W_pos_imu2ft;
    W_vel_fts2imu.segment(3 * i, 3) = (1 - contact(i)) * v0 - contact(i) * W_vel_imu2ft;
    W_height_ft(i) = (1 - contact(i)) * (p0(2) + W_pos_imu2ft(2));
  }

  Eigen::VectorXd y(28);
  y << W_pos_fts2imu, W_vel_fts2imu, W_height_ft;

  // Modify covariance considering contact states
  for(int i = 0; i < 4;i++) {
    Q_.block(6 + 3*i, 6 + 3*i, 3, 3) *= (1 + (1-contact[i]) * contact_suspect_);
    R_.block(12 + 3*i, 12 + 3*i, 3, 3) *= (1 + (1-contact[i]) * contact_suspect_);
    R_(24 + i, 24 + i) *= (1 + (1-contact[i]) * contact_suspect_);
  }

  /// Prediction
  xhat_ = A_ * xhat_ + B_ * W_acc_imu;
  P_ = A_ * P_ * A_.transpose() + Q_;

  /// Update
  Eigen::MatrixXd S = C_ * P_ * C_.transpose() + R_;
  Eigen::MatrixXd K = P_ * C_.transpose() * S.llt().solve(Eigen::MatrixXd(28, 28).setIdentity());
  Eigen::VectorXd ey = y - C_ * xhat_;

  xhat_ += K * ey;
  P_ = (Eigen::MatrixXd(18, 18).setIdentity() - K * C_) * P_;

  Eigen::Vector3d W_pos_imu = xhat_.head(3);
  Eigen::Vector3d W_vel_imu = xhat_.segment(3, 3);

  Kinematics::imu2Bd(rotation, W_ang_vel, W_pos_imu, W_vel_imu, W_pos_bd, W_vel_bd);
  est_log << W_pos_bd(0) << ", " << W_pos_bd(1) << ", " << W_pos_bd(2) << ", ";
  est_log << W_vel_bd(0) << ", " << W_vel_bd(1) << ", " << W_vel_bd(2) << "\n";

  // Revert covariance considering contact states
  for(int i = 0; i < 4;i++) {
    Q_.block(6 + 3*i, 6 + 3*i, 3, 3) /= (1 + (1-contact[i]) * contact_suspect_);
    R_.block(12 + 3*i, 12 + 3*i, 3, 3) /= (1 + (1-contact[i]) * contact_suspect_);
    R_(24 + i, 24 + i) /= (1 + (1-contact[i]) * contact_suspect_);
  }
}

void Estimator::estimate2(const Eigen::Matrix3d &rotation,
                         const Eigen::Vector3d &acc,
                         const Eigen::Vector3d &gyro,
                         const Eigen::Vector4i &contact,
                         const Eigen::VectorXd &joint_pos,
                         const Eigen::VectorXd &joint_vel,
                         Eigen::Vector3d &W_pos_bd,
                         Eigen::Vector3d &W_vel_bd){

//    Eigen::Vector3d acc_new = Eigen::Vector3d(acc.x(), acc.y(),0);
    Eigen::Vector3d W_acc_imu = rotation * (-acc) + Eigen::Vector3d(0, 0, -gravity);
    Eigen::Vector3d W_ang_vel = rotation * gyro;
//    Eigen::Vector3d W_acc_imu = Eigen::Vector3d::Zero();
//    Eigen::Vector3d W_ang_vel = Eigen::Vector3d::Zero();


    estimate(rotation, W_acc_imu, W_ang_vel, contact, joint_pos, joint_vel, W_pos_bd, W_vel_bd);
}

void Estimator::estimate(const Eigen::Vector3d& gyro,
                        const Eigen::Vector3d& acc,
                        const Eigen::Vector4i& contact,
                        const Eigen::VectorXd& joint_pos,
                        const Eigen::VectorXd& joint_vel,
                         const Eigen::VectorXd& joint_input,
                         Eigen::Vector3d& W_pos_bd,
                        Eigen::Vector3d& W_vel_bd) {
  complementaryFilter(gyro, acc);
  Eigen::Matrix3d rotation;
  Operation::quatToRotation(quat_, rotation);

  Eigen::Vector3d W_acc_imu = rotation * acc + Eigen::Vector3d(0, 0, -gravity);
  Eigen::Vector3d W_ang_vel = rotation * gyro;

  estimate(rotation, W_acc_imu, W_ang_vel, contact, joint_pos, joint_vel, W_pos_bd, W_vel_bd);
}

void Estimator::estimate(const Eigen::Vector3d& gyro,
                         const Eigen::Vector3d& acc,
                         const Eigen::VectorXd& joint_pos,
                         const Eigen::VectorXd& joint_vel,
                         const Eigen::VectorXd& joint_input,
                         Eigen::Vector3d& W_pos_bd,
                         Eigen::Vector3d& W_vel_bd) {

  Eigen::Vector4d knee_vel;
  knee_vel << joint_vel(1), joint_vel(4), joint_vel(7), joint_vel(10);
  Eigen::Vector4d knee_input;
  knee_input << joint_input(1), joint_input(4), joint_input(7), joint_input(10);

  Eigen::Vector4i contact;
  detectContact(knee_vel, knee_input, contact);

  estimate(gyro, acc, contact, joint_pos, joint_vel, joint_input, W_pos_bd, W_vel_bd);
}

void Estimator::reset() {
  Eigen::Vector3d init_pos(0, 0, height);
  Eigen::Vector4d init_quat(1, 0, 0, 0);
  quat_ = init_quat;

  Eigen::VectorXd init_joint(12);
  for(int i = 0; i < 4; i++)
    init_joint.segment(3*i, 3) << 0, init_angle, -2 * init_angle;

  xhat_.setZero(18);
  Eigen::Vector3d B_bd2imu(bd2imu[0], bd2imu[1], bd2imu[2]);
  xhat_.head(3) = init_pos + B_bd2imu;

  for (int i = 0; i < 4; i++) {
    xhat_.segment(6 + 3*i, 3) = init_pos + Kinematics::getFootPosition(i, Eigen::Matrix3d::Identity(), init_joint.segment(3*i, 3));
    xhat_(8 + 3*i) = 0;
  }

  eInt_.setZero();

  residual_prev_.setZero();
  momentum_hat_.setZero();
}
