#ifndef RBMOTORCONTROLLER_H
#define RBMOTORCONTROLLER_H

#include "RBDataType.h"
#include "RBCAN.h"

#include <math.h>

typedef struct _MOVE_JOINT_{
    double			RefAngleCurrent;	// reference to move at this step
    double			RefAngleCurrentOut;	// for display
    double			AngleCurrentOut;    // for display

    double			RefAngleDelta;		// reference of the past step
    double			RefAngleToGo;		// goal position - initial position
    double			RefAngleInitial;	// initial position
    unsigned long	GoalTimeCount;		// the time at which the goal is reached
    unsigned long	CurrentTimeCount;	// current time count
    char			MoveFlag;			// move flag

    int             Profile;
    // ---------------------
    float           STP_MaxAcc;
    float           STP_Speed;
    int             STP_Type;           // 0: reach target speed    1: not reach target speed
    int             STP_AccTimeCnt;
    float           STP_acctime;
    float           STP_goaltime;
    float           STP_acc;

    float           acctime;
    float           goaltime;

} MOVE_JOINT, *pMOVE_JOINT;


class RBMotorController
{
public:
    RBMotorController();

    // from DB----
    int     BOARD_ID;
    int     BOARD_TYPE;
    int     TOTAL_CHANNEL;
    int     MOTOR_CHANNEL;
    int     CAN_CHANNEL;
    int     ID_SEND_REF;
    int     ID_SEND_REF2;
    int     ID_RCV_ENC;
    int     ID_RCV_STAT;
    int     ID_RCV_INFO;
    int     ID_RCV_PARA;
    int     ID_SEND_GENERAL;


    int     ConnectionStatus;


    // Joint Variables--------
    double          PPR;    // Encoder pulse per one rotation(axis)
    int          TPC;    // Torque per current

    //--- DrPark
    double  homeJointOffset;
    double  homeJointOffset_mult;
    double  homeJointLift;
    double  homeJointReady;
    double  homeJointGround;

    mSTAT           CurrentStatus;

    // Control reference
    double          Reference;
    double          RefPos;
    double          RefPosOld;

    // Sensor information
    int             MeasuredCurrent;
    int             EncoderValue;
    double          CurrentPosition_Joint;
    double          CurrentPosition;
    float           MotorTemperature;
    float           BoardTemperature;
    double          CurrentVelocity;//added, diff here
    double          CurrentAcc;//added, diff here
    // -----------------------

    short   CurrentReference_mA;
    double  CurrentReference_A_low;
    double  CurrentReference_vel;
    double  CurrentReference_acc;
    float   Torque_Constant;



//    RB_JOINT    Joints[MAX_JOINT];
    MOVE_JOINT  MoveJoints;


    void    RBMC_AddCANMailBox();
    void    RBBoard_GetDBData(DB_MC db);
    void    RBBoard_SetCANIDs(int bno, int ch, int can_ch);
    void    RBMC_SetFrictionParam();

    void    RBBoard_ReferenceOutEnable(bool _refEnable);
    int     RBBoard_CANCheck(int _canr);
    int     RBBoard_RequestStatus(void);
    int     RBBoard_GetStatus(void);
    int     RBBoard_LoadDefaultValue(void);
    int     RBBoard_SetBoardNumber(int _newbno, int _canr);
    int     RBBoard_SetControlTime(float period_ms);
    int     RBBoard_RequestEncoder(int mode);
    int     RBBoard_RequestCurrent(void);

    int     RBJoint_ResetEncoder(int ch);
    int     RBJoint_EnableFETDriver(int ch, int enable);
    int     RBJoint_EnableFeedbackControl(int ch, int enable, int mode = 0);
    int     RBJoint_FindHome(int ch);
    int     RBJoint_EnableFrictionCompensation(int ch, int enable);
    int     RBJoint_SetMaxDuty(int ch, int duty);
    int     RBJoint_SetPositionCommandMode(int _mode, int _ch);
    int     RBJoint_ClearErrorFlag(int ch);

    int     RBBoard_RequestTemperature();
    int     RBBoard_ReadTemperature(void);
    int     RBBoard_PWMCommand2ch(int mode1, short duty1, int mode2, short duty2);
    int     RBBoard_SetControlMode(int _mode);
//    int     RBBoard_SendReference2ch(int ref1, int current_limit_T);
//    int     RBBoard_SendReference(int current_limit);
    int     RBBoard_SendReference2ch(int ref1, int enc_request_T);
    int     RBBoard_SendReference(int enc_request_time);



    int     RBBoard_SetErrorBound(int _ierror, int _berror, int _teeror);
    int     RBBoard_RequestParameter(int _para);



    void    RBJoint_SetMoveJoint(float angle, float timeMs, int mode);
    void    RBJoint_SetMoveJointSTrapi(float angle, float speed, int mode);
    void    RBJoint_MoveJoint();


    int     RB_InitDummy();
    int     RB_InitControl(unsigned int sol_duty = 12);

    int     RB_SolenoidControl(int mode);
    int     RB_CheckUVW();
    int     RB_DQAlign();
    int     RB_CurrentNulling();
    int     RB_ControlFrictionCompensation(int start_stop);
    int     RB_SetFrictionParam(int low_vel, int high_vel, int amp);

    int     RB_SetFrictionParam_1(int low_vel, int high_vel);
    int     RB_SetFrictionParam_2(int amp, int dead_amp);
    int     RB_RequestFrictionParam();

//    int     RBHome_RequestIndexPulse();
    int     RBHome_RequestHomeZeroOffset();
    int     RBHome_SetHomeZeroOffset(int offset, int now_zero = 0);



    int     RBFOC_RequestAutoTuneParams();
    int     RBFOC_SetAutoTuneParams(int at_curoffset, int at_inputvq);
    int     RBFOC_StartAutoTune();

    int     RBFOC_RequestGain();
    int     RBFOC_SetGain(int pgain, int igain, int mode);

    int     RBFOC_CurrentControl(int onoff);
    int     RBFOC_OpenLoopControl(int onoff, int inputV);


    int     RBFOC_RequestAutoTuneParams_POS();
    int     RBFOC_SetAutoTuneParams_POS(int at_posoffset, int at_inputcurrent);
    int     RBFOC_StartAutoTune_POS();

    int     RBFOC_RequestGain_POS();
    int     RBFOC_SetGain_POS(int pgain, int igain, int dgain, int mode);


    int     RBFOC_Measure_L();
    int     RBFOC_SineWaveTest(int start_stop, int amp, int period);

    int     RB_PWM_DeadTime(int deadtime_ns);

    int     RB_PVL_RESET();
    int     RB_SAVE_MU_DISK_OFFSET();
    int     RB_SPI_OFF();

    // uiuk added - encoder request time
    int     RB_Encoder_RequestTime(int enc_request_time);


    int     FrictionParam_LowVel;
    int     FrictionParam_HighVel;
    int     FrictionParam_Amp;

    int     Home_IndexPulse;
    int     Home_AbsPulse;
    int     Home_ZeroOffset;


    int     AutoTune_CurOffset;
    int     AutoTune_InputVq;
    int     AutoTune_PosOffset;
    int     AutoTune_InputCurrent;

    int     FOC_PGain;
    int     FOC_IGain;
    int     POS_PGain;
    int     POS_IGain;
    int     POS_DGain;


    bool    ReferenceOutEnable;

private:


};

#endif // RBMOTORCONTROLLER_H
