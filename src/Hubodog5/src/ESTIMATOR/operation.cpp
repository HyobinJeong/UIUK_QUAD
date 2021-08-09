//
// Created by legendarypyunny on 2020-07-28.
//
#include "operation.h"

Operation::Operation(const double dt):dt_(dt) {}

void Operation::quatToRotation(const Eigen::Vector4d& q, Eigen::Matrix3d& R) {
  R(0) = q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3];
  R(1) = 2 * q[0] * q[3] + 2 * q[1] * q[2];
  R(2) = 2 * q[1] * q[3] - 2 * q[0] * q[2];

  R(3) = 2 * q[1] * q[2] - 2 * q[0] * q[3];
  R(4) = q[0] * q[0] - q[1] * q[1] + q[2] * q[2] - q[3] * q[3];
  R(5) = 2 * q[0] * q[1] + 2 * q[2] * q[3];

  R(6) = 2 * q[0] * q[2] + 2 * q[1] * q[3];
  R(7) = 2 * q[2] * q[3] - 2 * q[0] * q[1];
  R(8) = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
}

Eigen::Vector3d Operation::quatToEul(const Eigen::Vector4d &quat) {
  double sinr_cosp = 2 * (quat(0) * quat(1) + quat(2) * quat(3));
  double cosr_cosp = 1 - 2 * (quat(1) * quat(1) + quat(2) * quat(2));
  double roll = std::atan2(sinr_cosp, cosr_cosp);

  double sinp = 2 * (quat(0) * quat(2) - quat(3) * quat(1));
  double pitch;
  if(std::abs(sinp) >= 1)
    pitch = std::copysign(PI / 2, sinp);
  else
    pitch = std::asin(sinp);

  double siny_cosp = 2 * (quat(0) * quat(3) + quat(1) * quat(2));
  double cosy_cosp = 1 - 2 * (quat(2) * quat(2) + quat(3) * quat(3));
  double yaw = std::atan2(siny_cosp, cosy_cosp);

  Eigen::Vector3d rpy(roll, pitch, yaw);
  return rpy;
}

Eigen::Vector4d Operation::quatProduct(const Eigen::Vector4d &a,
                                       const Eigen::Vector4d &b) {
  Eigen::Vector4d ab;
  ab(0) = a(0)*b(0) - a(1)*b(1) - a(2)*b(2) - a(3)*b(3);
  ab(1) = a(0)*b(1) + a(1)*b(0) + a(2)*b(3) - a(3)*b(2);
  ab(2) = a(0)*b(2) - a(1)*b(3) + a(2)*b(0) + a(3)*b(1);
  ab(3) = a(0)*b(3) + a(1)*b(2) - a(2)*b(1) + a(3)*b(0);

  return ab;
}

void Operation::rotationIntegration(Eigen::Matrix3d &rotation, double dt,
    const Eigen::Vector3d& angular_velocity) {
  const double norm = angular_velocity.norm();
  const double angle = norm*dt;
  if (angle < 1e-11) return;
  const double normInv = 1.0/norm;

  const double x = angular_velocity[0] * normInv;
  const double y = angular_velocity[1] * normInv;
  const double z = angular_velocity[2] * normInv;

  const double s = sin(angle);
  const double c = 1.0 - cos(angle);

  const double t2 = c*x*y;
  const double t3 = z*z;
  const double t4 = s*y;
  const double t5 = c*x*z;
  const double t6 = c*y*z;
  const double t7 = x*x;
  const double t8 = y*y;

  Eigen::Matrix3d expM;
  expM(0) = -c*t3-c*t8+1.0;
  expM(3) = t2-s*z;
  expM(6) = t4+t5;
  expM(1) = t2+s*z;
  expM(4) = -c*t3-c*t7+1.0;
  expM(7) = t6-s*x;
  expM(2) = -t4+t5;
  expM(5) = t6+s*x;
  expM(8) = -c*t7-c*t8+1.0;

  rotation *= expM;
}

Eigen::Vector3d Operation::matrixLog(
    const Eigen::Matrix3d &R) {
  double acosinput = (R.trace() - 1) / 2;
  Eigen::Matrix3d so3mat;
  Eigen::Vector3d omg;
  Eigen::Vector3d result;

  if(acosinput >= 1)
    so3mat.setZero();
  else if (acosinput <= -1) {
    if (std::abs(1 + R(2, 2)) > 1e-12)
      omg = (1 / sqrt(2 * (1 + R(2, 2))))
          * Eigen::Vector3d(R(0, 2), R(1, 2), 1 + R(2, 2));
    else if (std::abs(1 + R(1, 1)) > 1e-12)
      omg = (1 / sqrt(2 * (1 + R(1, 1))))
          * Eigen::Vector3d(R(0, 1), 1 + R(1, 1), R(2, 1));
    else
      omg = (1 / sqrt(2 * (1 + R(0, 0))))
          * Eigen::Vector3d(1 + R(0, 0), R(1, 0), R(2, 0));
    so3mat << 0, -omg(2), omg(1),
              omg(2), 0, -omg(0),
              -omg(1), omg(0), 0;
  } else
    so3mat = acos(acosinput) * (1 / ( 2 * sin(acos(acosinput)) )) * (R - R.transpose());

  result(0) = so3mat(2,1);
  result(1) = so3mat(0,2);
  result(2) = so3mat(1,0);
  return result;
}

void Operation::vstack(
    Eigen::SparseMatrix<double> &A,
    const Eigen::SparseMatrix<double> &B) {
  Eigen::SparseMatrix<double> C(A.rows() + B.rows(), A.cols());
  C.reserve(A.nonZeros() + B.nonZeros());
  for(Eigen::Index c=0; c < A.cols(); ++c)
  {
    C.startVec(c); // Important: Must be called once for each column before inserting!
    for(Eigen::SparseMatrix<double>::InnerIterator itA(A, c); itA; ++itA)
      C.insertBack(itA.row(), c) = itA.value();
    for(Eigen::SparseMatrix<double>::InnerIterator itB(B, c); itB; ++itB)
      C.insertBack(itB.row()+A.rows(), c) = itB.value();
  }
  C.finalize();
  A = C;
}

Eigen::VectorXd Operation::NormalRandom (int n) {
  std::default_random_engine random_engine(std::chrono::steady_clock::now().time_since_epoch().count());
  std::normal_distribution<double> normal_distribution(0,1);

  Eigen::VectorXd random = Eigen::VectorXd::Zero(n);
  for (int i = 0; i < n; i++)
    random[i] = normal_distribution(random_engine);

  return random;
}

void Operation::getSwingRef(double duration,
                            double elapsed,
                            const Eigen::Vector3d &start,
                            const Eigen::Vector3d &middle,
                            const Eigen::Vector3d &final,
                            Eigen::Vector3d& p_target,
                            Eigen::Vector3d& d_target,
                            Eigen::Vector3d& a_target) {
  Eigen::Vector3d mid_vel = (final - start) / duration;

  Eigen::MatrixXd pos, vel, acc;
  drawBezier(start, middle, final, mid_vel, duration, pos, vel, acc);

  int idx = int((pos.cols() - 1) * elapsed / duration);
  p_target = pos.col(idx);
  d_target = vel.col(idx);
  a_target = acc.col(idx);
}

void Operation::drawBezier(const Eigen::Vector3d &start,
                           const Eigen::Vector3d &middle,
                           const Eigen::Vector3d &final,
                           const Eigen::Vector3d &mid_vel,
                           double duration,
                           Eigen::MatrixXd &pos,
                           Eigen::MatrixXd &vel,
                           Eigen::MatrixXd &acc) {
  Eigen::VectorXd times;
  times.setLinSpaced(50, 0, 1);

  int order = 5;
  double s_r = 0.5;
  Eigen::Vector3d alpha1_2 = middle - mid_vel * duration * s_r / order;
  Eigen::Vector3d alpha1_1 = 2 * alpha1_2 - middle;

  Eigen::MatrixXd alpha1;
  alpha1.setZero(3, order + 1);
  for (int i = 0; i < order - 2; i++)
    alpha1.col(i) = start;
  alpha1.col(order - 2) = alpha1_1;
  alpha1.col(order - 1) = alpha1_2;
  alpha1.col(order) = middle;

  Eigen::Vector3d alpha2_1 = middle + mid_vel * duration * (1 - s_r) / order;
  Eigen::Vector3d alpha2_2 = 2 * alpha2_1 - middle;
  Eigen::MatrixXd alpha2;
  alpha2.setZero(3, order + 1);
  alpha2.col(0) = middle;
  alpha2.col(1) = alpha2_1;
  alpha2.col(2) = alpha2_2;
  for (int i = 3; i < order + 1; i++)
    alpha2.col(i) = final;

  Eigen::MatrixXd position1, position2;
  Eigen::MatrixXd velocity1, velocity2;
  Eigen::MatrixXd acc1, acc2;
  getBezierPoints(alpha1, times, position1, velocity1, acc1);
  getBezierPoints(alpha2, times, position2, velocity2, acc2);
  velocity1 /= (duration * s_r);
  velocity2 /= (duration * (1 - s_r));
  acc1 /= ((duration * s_r) * (duration * s_r));
  acc2 /= ((duration * (1 - s_r)) * (duration * (1 - s_r)));

  pos.setZero(3, position1.cols() + position2.cols());
  pos << position1 , position2;
  vel.setZero(3, velocity1.cols() + velocity2.cols());
  vel << velocity1, velocity2;
  acc.setZero(3, velocity1.cols() + velocity2.cols());
  acc << acc1, acc2;

}

void Operation::getBezierPoints(const Eigen::MatrixXd &points,
                                const Eigen::VectorXd &times,
                                Eigen::MatrixXd &position,
                                Eigen::MatrixXd &velocity,
                                Eigen::MatrixXd &acc) {
  position.setZero(3, times.size());
  velocity.setZero(3, times.size());
  acc.setZero(3, times.size());

  int order = points.cols() - 1;
  Eigen::VectorXd ones(times.size());
  ones.setOnes();

  for(int j = 0; j < 3; j++) {
    for (int i = 0; i < order + 1; i++) {
      Eigen::VectorXd basis = times.array().pow(i) * (ones - times).array().pow(order - i);
      double coef = factorial(order) / (factorial(i) * factorial(order - i));

      position.row(j) += coef * points(j, i) * basis.transpose();
    }

    for(int i = 0; i < order; i++) {
      Eigen::VectorXd basis = times.array().pow(i) * (ones - times).array().pow(order-1 - i);
      double coef = factorial(order) / (factorial(i) * factorial(order-1 -i)) ;

      velocity.row(j) += coef * (points(j, i+1) - points(j, i)) * basis.transpose();
    }

    for(int i = 0; i < order - 1; i++) {
      Eigen::VectorXd basis = times.array().pow(i) * (ones - times).array().pow(order-2 - i);
      double coef = factorial(order) / (factorial(i) * factorial(order-2 -i));

      acc.row(j) += coef * (points(j, i + 2) - 2 * points(j, i + 1) + points(j, i)) * basis.transpose();

    }
  }

}

//bool Operation::solveQP(const Eigen::SparseMatrix<double> &P,
//                        const Eigen::SparseMatrix<double> &A,
//                        const Eigen::SparseMatrix<double> &G,
//                        const Eigen::VectorXd &q,
//                        const Eigen::VectorXd &b,
//                        const Eigen::VectorXd &h,
//                        double &qp_time,
//                        qp_int **permut,
//                        Eigen::VectorXd &solution)  {

//  Eigen::SparseMatrix<double, Eigen::ColMajor, qp_int> P_copy(P);
//  Eigen::SparseMatrix<double, Eigen::ColMajor, qp_int> A_copy(A);
//  Eigen::SparseMatrix<double, Eigen::ColMajor, qp_int> G_copy(G);
//  P_copy.makeCompressed();
//  A_copy.makeCompressed();
//  G_copy.makeCompressed();

//  // QP Swift
//  qp_real *Ppr = P_copy.valuePtr();
//  qp_real *Apr = A_copy.valuePtr();
//  qp_real *Gpr = G_copy.valuePtr();

//  qp_int *Pir = P_copy.innerIndexPtr();
//  qp_int *Air = A_copy.innerIndexPtr();
//  qp_int *Gir = G_copy.innerIndexPtr();

//  qp_int *Pjc = P_copy.outerIndexPtr();
//  qp_int *Ajc = A_copy.outerIndexPtr();
//  qp_int *Gjc = G_copy.outerIndexPtr();

//  qp_real* q_ = const_cast<qp_real *>(q.data());
//  qp_real* b_ = const_cast<qp_real *>(b.data());
//  qp_real* h_ = const_cast<qp_real *>(h.data());

//  QPswift *qpswift = QP_SETUP(q.size(), h.size(), b.size(),
//                     Pjc, Pir, Ppr,
//                     Ajc, Air, Apr,
//                     Gjc, Gir, Gpr,
//                     q_, h_, b_,
//                     0, permut);

//  qp_int exit_code = QP_SOLVE(qpswift);

//  qp_time += qpswift->stats->tsetup + qpswift->stats->tsolve;
//  solution = Eigen::Map<Eigen::VectorXd>(qpswift->x, P_copy.cols(), 1);

//  QP_CLEANUP(qpswift);

//  double *solution_ptr = solution.data();
//  for ( int j = 0; j < q.size(); j++)
//    if(std::isnan(*solution_ptr++))
//      return false;

//  return true;
//}


