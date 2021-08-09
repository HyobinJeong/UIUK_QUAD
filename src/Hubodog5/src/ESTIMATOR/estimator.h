#ifndef ESTIMATOR_H
#define ESTIMATOR_H

#include <Eigen/Core>
#include <Eigen/Dense>


#include "kinematics.h"
#include "operation.h"

class Estimator {
public :
    Estimator(const Eigen::Vector3d &init_pos,
               const Eigen::VectorXd &init_joint,
               const Eigen::Vector4d &init_quat,
               double dt);

    Estimator(double dt);

    void Init(const Eigen::Vector3d &init_pos,
              const Eigen::VectorXd &init_joint,
              const Eigen::Vector4d &init_quat);

    ~Estimator();

    Eigen::Vector4d getQuaternion() {return quat_;}

    void setIMUPIGain(double Kp, double Ki) {
      imu_Kp = Kp;
      imu_Ki = Ki;
    }

    Eigen::Vector4d getQuat(){
        return quat_;
    }

    void setCovariance(double imu_pos_cov,
                       double imu_vel_cov,
                       double ft_pos_cov,
                       double enc_pos_cov,
                       double enc_vel_cov,
                       double ft_height_cov);

    void complementaryFilter(const Eigen::Vector3d &gyro,
                             const Eigen::Vector3d &acc);

    void reset_complementaryFilter(const Eigen::Vector4d &_init_quat);

    void detectContact(const Eigen::Vector4d &knee_vel,
                       const Eigen::Vector4d &knee_input,
                       Eigen::Vector4i& contact);

    void estimate(const Eigen::Matrix3d& rotation,
                  const Eigen::Vector3d& W_acc_imu,
                  const Eigen::Vector3d& W_ang_vel,
                  const Eigen::Vector4i& contact,
                  const Eigen::VectorXd& joint_pos,
                  const Eigen::VectorXd& joint_vel,
                  Eigen::Vector3d& W_pos_bd,
                  Eigen::Vector3d& W_vel_bd);

    void estimate(const Eigen::Vector3d& gyro,
                  const Eigen::Vector3d& acc,
                  const Eigen::Vector4i& contact,
                  const Eigen::VectorXd& joint_pos,
                  const Eigen::VectorXd& joint_vel,
                  const Eigen::VectorXd& joint_input,
                  Eigen::Vector3d& W_pos_bd,
                  Eigen::Vector3d& W_vel_bd);

    void estimate(const Eigen::Vector3d& gyro,
                  const Eigen::Vector3d& acc,
                  const Eigen::VectorXd& joint_pos,
                  const Eigen::VectorXd& joint_vel,
                  const Eigen::VectorXd& joint_input,
                  Eigen::Vector3d& W_pos_bd,
                  Eigen::Vector3d& W_vel_bd);
    void estimate2(const Eigen::Matrix3d &rotation,
                     const Eigen::Vector3d &acc,
                     const Eigen::Vector3d &gyro,
                     const Eigen::Vector4i& contact,
                     const Eigen::VectorXd &joint_pos,
                     const Eigen::VectorXd &joint_vel,
                     Eigen::Vector3d &W_pos_bd,
                     Eigen::Vector3d &W_vel_bd);

    void reset();

private :
    double dt_;
    int contact_suspect_ = 100;

    /// Complementary Filter
    Eigen::Vector4d quat_;
    Eigen::Vector3d eInt_;
    double imu_Kp = 0.0;
    double imu_Ki = 0;

    /// Contact Detection
    Eigen::Vector4d residual_prev_;
    Eigen::Vector4d momentum_hat_;
    Eigen::Vector4d contact_Ki_;
    double calf_I_;
    double contact_threshold_;

    /// Kalman Filter
    Eigen::VectorXd xhat_;

    Eigen::MatrixXd A_;
    Eigen::MatrixXd B_;
    Eigen::MatrixXd C_;

    Eigen::MatrixXd Q_;
    Eigen::MatrixXd P_;
    Eigen::MatrixXd R_;

    std::ofstream quat_log;
    std::ofstream est_log;
    std::ofstream contact_log;

};  // class Estimator
#endif // ESTIMATOR_H
