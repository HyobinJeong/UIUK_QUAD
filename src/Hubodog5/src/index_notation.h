#ifndef INDEX_NOTATION_H
#define INDEX_NOTATION_H

enum JointSequentialNumberBYNAME
{
    HRR, HRP, HRK,
    HLR, HLP, HLK,
    FRR, FRP, FRK,
    FLR, FLP, FLK,
    NO_OF_JOINTS
};

enum LegSequence
{
    HR, HL, // hind right-left
    FR, FL, // front right-left
    NO_OF_LEGS
};

enum QUADGAIT
{
    NoTask,
    OneJointTest,
    OneLegTest,
    DemoMotion,
    Dance01,
    TrotTest,
    HoppingTest,
    Backflip,
    Flyingtrot,
    ComControl,
    Standing,
    MPC,
    QP_Standing,
};

enum COMMAND
{
    NO_ACT,
    READYPOS,
    MOTION_START,
    MOTION_STOP,
    MOTOR_LOCK,
    MOTOR_OFF,
    MOUNTPOS,
    GROUNDPOS,
    GREADYPOS,
    HOMMINGPOS,
    JOGMODE,
    INCGAIN,
    INCGAIN2,
    MOTION_BACKFLIP,
    MOTION_BACKFLIP_H,
    MOTION_FALLED,
    MOTION_FALLRECOVER,
    MOTION_JUMP,
    MOTION_FALLING

};

#endif // INDEX_NOTATION_H
