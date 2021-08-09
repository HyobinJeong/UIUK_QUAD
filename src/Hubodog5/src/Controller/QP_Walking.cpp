#include "hubodog5_general.h"
#include "func_sensor.h"
#include "RBSharedMemory.h"
#include "ESTIMATOR/estimator.h"

extern RBMotorController   _DEV_MC[MAX_MC];
extern int MY_CONTROL_FB_GAIN[12];
extern int MY_CONTROL_FF_GAIN[12];

QuadJoint hubodog5_general::QP_Standing(){

    Vector3d x_des, x_real;
    Matrix3d R_des, R_real;

    Vector3d foot_HR, foot_HL, foot_FR, foot_FL;


    // Dynamics



}
