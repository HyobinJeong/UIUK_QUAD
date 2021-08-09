#ifndef FUNC_SENSOR_H
#define FUNC_SENSOR_H

#include "hubodog5_general.h"

void imu_zero();
double GetOneJointPos(int idx);
double GetOneJointPosRef(int idx);
double GetOneJointVel(int idx);
double GetOneJointCurrent(int idx);
QuadJoint GetAllJointPos();
QuadJoint GetAllJointPosRef();
QuadJoint GetAllJointVel();
QuadJoint GetAllJointCurrent();
QuadJoint GetAllJointAcc();
void set_ref_data();
void get_sensor_data();
void SetOneJointRef(double angle, int idx);
void SetOneCurrentRef(double current, int idx);
QuadJoint vec2QJ(double vector[12]);
double *QJ2vec(QuadJoint QJ);
void SetAllJointRef(QuadJoint JointRef);
void SetAllCurrentRef(QuadJoint Cref);
QuadJoint Torque2Current(QuadJoint torque);
double SetOneCurrent(double torque);

#endif // FUNC_SENSOR_H
