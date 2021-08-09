#ifndef RB_SHARED_MEMORY_H
#define RB_SHARED_MEMORY_H

#define RBCORE_SHM_NAME_REFERENCE       "RBCORE_SHARED_MEMORY_REFERENCE"
#define RBCORE_SHM_NAME_SENSOR          "RBCORE_SHARED_MEMORY_SENSOR"
#define RBCORE_SHM_NAME_COMMAND         "RBCORE_SHARED_MEMORY_COMMAND"

#define MAX_JOINT   1
#define MAX_MC      12
#define MAX_LEG      4
#define MAX_FT      0

#define MAX_MANUAL_CAN          10
#define MAX_COMMAND_DATA        10

#define MOTOR_1CH               1
#define MOTOR_2CH               2


#define COMMAND_CANID           0x10    //0x01
#define SENSOR_REQUEST_CANID    0x03    //0x02
#define RT_TIMER_PERIOD_MS      2.0
//#define RT_TIMER_PERIOD_MS      1.25       // uiuk modified

#define RBCORE_PODO_NO          0

#define ENABLE					1
#define DISABLE					0

#define RBCORE_PI               3.141592f

extern int  _IS_WORKING;
extern int  _NO_OF_MC;
extern int  _IS_NEW_CAN_DATA;


typedef enum{
    MANUALCAN_NEW = 0,
    MANUALCAN_EMPTY,
    MANUALCAN_WRITING
} MANUALCAN_STATUS;
typedef	struct _MANUAL_CAN_{
    unsigned char		channel;
    unsigned int		id;
    unsigned char		data[8];
    unsigned char		dlc;
    MANUALCAN_STATUS	status;
} MANUAL_CAN;


typedef union{
    struct{
        unsigned    FET:1;	 	// FET ON   //
        unsigned    RUN:1;		// Control ON
        unsigned    INIT:1;     // Init Process Passed  //
        unsigned    MOD:1;		// Control Mode
        unsigned    NON_CTR:1;		// Nonius Count err //
        unsigned    BAT:1;      // Low Battery //
        unsigned    CALIB:1;    // Calibration Mode //
        unsigned    MT_ERR:1;      // Reply Status //

        unsigned    JAM:1;		// JAM Error
        unsigned    CUR:1;		// Over Current Error
        unsigned    BIG:1;		// Big Position Error
        unsigned    INP:1;      // Big Input Error
        unsigned    FLT:1;		// FET Driver Fault Error //
        unsigned    TMP:1;      // Temperature Error //
        unsigned    PS1:1;		// Position Limit Error (Lower) ////
        unsigned    PS2:1;		// Position Limit Error (Upper)

        unsigned    rsvd:8;
    }b;
    unsigned char B[3];
}mSTAT;


typedef struct _ENCODER_SENSOR_
{
    int     BoardConnection;
    double   CurrentReference;
    double   CurrentPosition;
    double   CurrentVelocity;
}ENCODER_SENSOR;
typedef struct _IMU_SENSOR_
{
    int     BoardConnection;
    float   AccRoll;
    float   AccPitch;
    float   CFRoll;
    float   CFPitch;
    float   Roll;
    float   Pitch;
    float   Yaw;
    float   RollVel;
    float   PitchVel;
    float   YawVel;
    float   AccX;
    float   AccY;
    float   AccZ;
    double  Q[4];
    float   Roll_AHRS;
    float   Pitch_AHRS;
    float   Yaw_AHRS;
    int     NO_RESPONSE_CNT;
    int     RESET_FLAG;
}IMU_SENSOR;
typedef struct _FT_SENSOR_
{
    int     BoardConnection;
    float   Mx;
    float   My;
    float   Mz;
    float   Fx;
    float   Fy;
    float   Fz;
    float   RollVel;
    float   PitchVel;
    float   Roll;
    float   Pitch;
}FT_SENSOR;

typedef struct _COMMAND_STRUCT_
{
    int     USER_COMMAND;
    char    USER_PARA_CHAR[MAX_COMMAND_DATA];
    int	    USER_PARA_INT[MAX_COMMAND_DATA];
    float   USER_PARA_FLOAT[MAX_COMMAND_DATA];
    double   USER_PARA_DOUBLE[MAX_COMMAND_DATA];
} COMMAND_STRUCT, *pCOMMAND_STRUCT;

typedef struct _USER_COMMAND_
{
    int             COMMAND_TARGET;
    COMMAND_STRUCT  COMMAND_DATA;
} USER_COMMAND, *pUSER_COMMAND;


//======================================
// Reference Shared Memory <Write Only>
typedef struct _RBCORE_SHM_
{
    mSTAT           MCStatus[MAX_MC][MOTOR_2CH];
    double           JointReference[MAX_MC][MOTOR_2CH];
    IMU_SENSOR      IMU[2];

    float Vin;
    float Iin;

    int             CAN_Enabled;
    int             REF_Enabled;
    int             SEN_Enabled;
    int             ENC_Enabled;
    double          qPel[4];

    COMMAND_STRUCT  COMMAND;
    bool NEWCOMMAND;
    MANUAL_CAN		ManualCAN[MAX_MANUAL_CAN];

    bool isFalled;
    bool isStopping;
    bool incGain_flag = false;

    //------ custom variables -------
    double Walking_period;
    bool jog_joint[12];
    bool isSimulation, isIMUAlwaysZero;
    bool RaisimSimulation;
    int  RaisimGainState;
    int  RaisimResetFlag;
    double Kp[3], Kd[3], pf_ref_init[12];
    double Kp_j[3], Kd_j[3];
    double pf_real[48], pf_ref[48]; // pos[12]-vel[12]-acc[12]-force[12]
    double pcom_real[12], pcom_ref[12];
    double joint_info[48], joint_info_ref[48], joint_info_filtered[48]; // pos[12]-vel[12]-acc[12]-current[12]
    double joint_IK[12];
    int PCON[12], CCON[12];
    bool idx_legcon[4];
    double z0, x0;
    bool contact_flag[4], contact_flag_test[4];
    bool plot_play_ON, plot_timer_zero;
    bool OneLegTest_ON, OneLegTest_OFF;
    double demo_variable[7];
    bool task_legcon[4];
    double trot_variable_int[20];
    double trot_variable_double[30];
    double hopping_variable_int[20];
    double hopping_variable_double[20];
    double backflip_variable_int[20];
    double backflip_variable_double[20];
    double control_variable_int[20];
    double control_variable_double[20];
    int flyingtrot_variable_int[20];
    double flyingtrot_variable_double[20];
    double Cmax;
    bool isJointFilter;
    double frq_JointFilter;
    double frq_IMUFilter;
    double alpha_IMUFilter;
    int    btn_IMUFilter;
    char CustomFileName[30];
    double roll_ofs, pitch_ofs;

    double comcon_variable_int[20];
    double comcon_variable_double[20];
    double comcon_variable_plot[30];

    bool            savedata_ON, joy_ON;
    double          joy_dpCOM[4], joy_dRPY[3];
    int             joy_up, joy_down, joy_left, joy_right;
    int             joy_btn_a, joy_btn_b, joy_btn_x, joy_btn_y;
    int             joy_state;
    double          joy_Orientation[3];

    // MPC part
//    double          mpc_state[24];
//    double          mpc_state_s[24];
//    double          mpc_tsolve, mpc_tsim, mpc_timer_total, mpc_timer_setup, mpc_tCCS, mpc_tclean;
//    int             mpc_iter, mpc_cnt, mpc_flag;
//    double          mpc_GRF[12], mpc_gain, dpf_gain, mpc_dpf[12];
//    double          customWEIGHT[12], customWEIGHT_pow[7];
//    bool            MPCfilter_ON, MPC_ON;
//    double          MPCfilter_frq;

//    double          swing_ZT[2];
//    double          custom_Tsmpc;
//    int             custom_Nhorizon, custom_MPCfrq;
//    double          pf_CP[12], sim_time, uref_gain;
//    int             control_type;
//    double          foot_ref[12];
//    bool            istimerON, isGaitChanged;
//    bool            plot_timer_zero_2, plot_play_ON_2;
//    double          plot_timer_started;
//    double          acc2trq_fitting[3], extForce[12], acc2trq_coeff, contact_thres;
//    double          COMbias[3];
//    bool            stepping_ON;
//    int             stepping_num;
//    int             PCON_gain;
//    double          PCON_gain_Tincrease;

}RBCORE_SHM, *pRBCORE_SHM;
//======================================


//typedef enum _COMMAND_SET_{
//    NO_ACT = 0,
//    // For process handle
//    DAEMON_PROCESS_CREATE,
//    DAEMON_PROCESS_KILL,
//    // For initialize
//    DAEMON_INIT_CHECK_DEVICE,
//    DAEMON_INIT_FIND_HOME,
//    DAEMON_INIT_FET_ONOFF,
//    DAEMON_INIT_CONTROL_ONOFF,
//    DAEMON_INIT_SET_FINGER_MODIFIER,
//    // For attribute
//    DAEMON_ATTR_SWITCHING_MODE,
//    DAEMON_ATTR_FRICTION_COMPENSATION,
//    DAEMON_ATTR_CONTROL_MODE,
//    // For sensor
//    DAEMON_SENSOR_ENCODER_RESET,
//    DAEMON_SENSOR_ENCODER_ONOFF,
//    DAEMON_SENSOR_SENSOR_ONOFF,
//    DAEMON_SENSOR_FT_NULL,
//    // For motion
//    DAEMON_MOTION_REF_ONOFF,
//    DAEMON_MOTION_MOVE,
//    DAEMON_MOTION_GAIN_OVERRIDE,
//    DAEMON_MOTION_ERROR_CLEAR,
//    // For CAN
//    DAEMON_CAN_ENABLE_DISABLE,

//    DAEMON_SENSOR_IMU_NULL,
//} COMMAND_SET;

#endif // RB_SHARED_MEMORY_H

