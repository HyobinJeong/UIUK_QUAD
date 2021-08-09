#ifndef FUNC_GENERAL_H
#define FUNC_GENERAL_H

#include "hubodog5_struct.h"
#include "BasicMath.h"

#define PI			3.141592653589793
#define D2R			1.745329251994330e-2
#define R2D			5.729577951308232e1


//MatrixNd Tmat(Matrix3d R, Vector3d p);
//Matrix3d getOneLegJacobian(Vector3d EncPos, Vector3d EncVel, int LR);
//Vector3d FK_OneLeg_local(Vector3d leg_angle, int leg_idx);
//Vector3d IK_OneLeg(Vector3d foot_pos, int leg_idx);
//QuadJoint IK_FullBody(RobotState state);

Matrix3d quat2R(Vector4d quat);
Matrix3d Euler2R(Vector3d e);
Matrix3d rotate_3D(Vector3d axis, double angle);
Vector4d R2Quat(Matrix3d m);
MatrixNd diagonalize(VectorNd vector);
MatrixNd Tmat(Vector3d p, Matrix3d R);
MatrixNd Tmat_inv(MatrixNd Tmat);
Matrix3d vec2hat(Vector3d vec);
Vector3d hat2vec(Matrix3d mat);
MatrixNd mat_reshape(MatrixNd mat, int row, int col);
MatrixNd mat_repeat(MatrixNd mat, int row, int col);
MatrixNd mat_kronecker(MatrixNd A, MatrixNd B);
Vector3d bezier_calc_3rd(double t_now, double t_end, Vector3d xdxddx_start, Vector3d xdxddx_end);
Vector3d bezier_calc_2rd(double t_now, double t_end, Vector3d xdxddx_start, Vector3d xdxddx_end);
Vector3d bezier_calc(double t_now, double t_end, Vector3d xdxddx_start, Vector3d xdxddx_end);
VectorNd bezier_coeff(VectorNd xdxddx, int order);
double bezier_polynomial_x(VectorNd coeff, double t);
double bezier_polynomial_dx(VectorNd coeff, double t);
double bezier_polynomial_ddx(VectorNd coeff, double t);
int factorial(int a);
Vector3d Lowpass_filter_3d(Vector3d input, Vector3d output, double f_cut, double ts);
Vector3d Highpass_filter_3d(Vector3d input, Vector3d input_pre, Vector3d output, double f_cut, double ts);
double Lowpass_filter(double input, double output, double f_cut, double ts);
double Highpass_filter(double input, double input_pre, double output, double f_cut, double ts);
Vector3d calc_3rd(double t_now, double t_e, Vector3d p_init, Vector3d p_end, double ts);

#endif // FUNC_GENERAL_H
