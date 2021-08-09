#ifndef FUNC_UIUK_H
#define FUNC_UIUK_H

//#include "hubodog5_struct.h"
//#include "BasicMath.h"
#include "func_general.h"
#include "hubodog5_general.h"
#include "RBMotorController.h"


Vector3d MovePos(double t_now, double t_end, Vector3d pos_start, Vector3d pos_end);
Vector3d StanceLeg(double t_now, double t_end, Vector3d pos_start, Vector3d pos_end);
Vector3d SwingLeg(double t_now, double t_end, Vector3d pos_start, Vector3d pos_end, double step_z);
Vector3d StanceLegV(double t_now, double t_end, Vector3d pos_start, Vector3d pos_end, Vector3d vel_start, Vector3d vel_end);
Vector3d SwingLegV(double t_now, double t_end, Vector3d pos_start, Vector3d pos_end, Vector3d vel_start, Vector3d vel_end, double step_z);
Vector3d SwingLeg_J(double t_now, double t_end, Vector3d pos_start, Vector3d pos_end, double step_z, double t_jump, double ofs_jump);
Vector3d FootStepRotation(double step_r, int lnum);
Vector3d ComYawCompensation(double angle, int lnum);
Vector3d FootXY_Swing_trajectory(double _t_foot_now, double _real_t_step, double _x_start, double _x_end, double _touch_vel, Vector3d _X_dX_ddX_Foot_old);
Vector3d FootZ_Swing_trajectory(double _t_foot_now, double _real_t_step, double _z_start, double _z_end, double _step_z, Vector3d _Z_dZ_ddZ_Foot_old);
Vector3d FootZ_Stance_Jump_trajectory(double _t_foot_now, double _real_t_step, double _z_start, double _z_push_vel, Vector3d _Z_dZ_ddZ_Foot_old);
Vector3d FootZ_Swing_Jump_trajectory(double _t_foot_now, double _real_t_step, double _z_start, double _z_end, double _step_z, Vector3d _Z_dZ_ddZ_Foot_old);
Vector3d FootXY_trajectory(double _t_foot_now, double _real_t_step, double _x_start, double _x_end, double _off_vel, Vector3d _X_dX_ddX_Foot_old);

void BackFlipMotion();


#endif // FUNC_UIUK_H
