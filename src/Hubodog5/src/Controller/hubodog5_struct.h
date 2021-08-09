#ifndef HUBODOG5_STRUCT_H
#define HUBODOG5_STRUCT_H

#include "rbdl/rbdl.h"
#include "Eigen/Geometry"
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

struct QuadJoint
{
    union{
        struct{
            double HRR, HRP, HRK;
            double HLR, HLP, HLK;
            double FRR, FRP, FRK;
            double FLR, FLP, FLK;
        };
        double QJ_Array[12];
    };

};

typedef struct _PointState_
{
    Eigen::Vector3d pos;
    Eigen::Vector3d vel;
    Eigen::Vector3d acc;
    Eigen::Vector3d force;
}PointState;

struct QuadConfig
{

    PointState HR;
    PointState HL;
    PointState FR;
    PointState FL;

};

struct JointAngleInfo
{
    QuadJoint pos;
    QuadJoint pos_ref;
    QuadJoint vel;
    QuadJoint acc;
    QuadJoint current;
};

struct IMUInfo
{
    Vector3d angle;
    Vector4d quat;
    Vector4d quat_RPY;
    Vector3d vel;
    Vector3d acc;
};

struct SensorInfo
{
    JointAngleInfo Encoder;
    IMUInfo IMU;
};

struct RobotState
{
    PointState torso;
    Matrix3d rotate;
    PointState COM;
    QuadConfig hip;
    QuadConfig foot;
};




#endif // HUBODOG5_STRUCT_H
