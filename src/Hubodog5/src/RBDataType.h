#ifndef RBDATATYPE_H
#define RBDATATYPE_H

#include <unistd.h>
#include <math.h>
#include "RBSharedMemory.h"


typedef	unsigned int	uint;
typedef unsigned char	uchar;
typedef	unsigned long	ulong;

typedef class RBCAN     *pRBCAN;
extern pRBCAN           canHandler;

//======================================================
// Database data type for General
typedef struct _DB_GENERAL_{
    int     VERSION;
    int     NO_OF_MC;           // motor controller
    int     NO_OF_FT;           // ft sensor
}DB_GENERAL;

// Database data type for Motor Controller
typedef struct _DB_JOINT_{
    double  HARMONIC;
    double  PULLY_DRIVE;
    double  PULLY_DRIVEN;
    double  ENCODER_RESOLUTION;
    double  PPR;
}DB_JOINT;

typedef struct _DB_MC_{
    int     BOARD_ID;
    int     BOARD_TYPE;
    int     MOTOR_CHANNEL;
    int     CAN_CHANNEL;
    int     ID_SEND_REF;
    int     ID_SEND_REF2;
    int     ID_RCV_ENC;
    int     ID_RCV_STAT;
    int     ID_RCV_INFO;
    int     ID_RCV_PARA;
    int     ID_SEND_GENERAL;
    DB_JOINT    JOINTS[MAX_JOINT];
}DB_MC;
//======================================================




//======================================================
typedef struct _RBCORE_JOINT_
{
    // Joint information from a board
    int             GainKp;
    int             GainKi;
    int             GainKd;
    int             EncoderResolution;
    int             PositiveDirection;
    int				AutoScale;
    int             Deadzone;
    int             SearchDirection;
    int             HomeSearchMode;
    int             LimitRevolution;
    double          OffsetAngle;
    double          LowerPositionLimit;
    double          UpperPositionLimit;
    int             MaxAcceleration;
    int             MaxVelocity;
    int             MaxPWM;
    int             MaxAccelerationDuringHome;
    int             MaxVelocityDuringHome;
    int				MaxVelocityDuringGoOffset;
    int             JAMmsTime;
    int             PWMmsTime;
    int             PWMDuty;
    int             JAMDuty;
    int             CurrentErrorLimitValue;
    int             BigInputErrorLimitValue;
    int             EncoderErrorLimitValue;

    double          Harmonic;
    double          PullyDrive;
    double          PullyDriven;
    double          PPR;    // Encoder pulse per one rotation(axis)
    double          ReductionRatio;
    double          FrictionParam1;
    double          FrictionParam2;

    // Joint status
    mSTAT           CurrentStatus;
    int             ControlMode;
    int             TorqueMode;

    // Control reference
    double          Reference;
    double          RefPos;
    double          RefVel;
    double          RefVelConversionError;
    double          RefCurrent;
    double          RefPWM;
    double          RefPosOld;
    double          RefVelOld;
    double          RefCurrentOld;
    double          RefPWMOld;
    double          RefPosPreDefined[3];
    double          RefDefaultMotion[5];

    // Sensor information
    int             MeasuredCurrent;
    int             EncoderValue;
    double          CurrentPosition;
    double          CurrentVelocity;
    double          CurrentCurrent;
    float           Temperature;
} RB_JOINT, *pRB_JOINT;
//======================================================
#endif // RBDATATYPE_H
