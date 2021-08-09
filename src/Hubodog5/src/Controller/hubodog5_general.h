#ifndef HUBODOG5_GENERAL_H
#define HUBODOG5_GENERAL_H

#include "main_controller.h"
#include "RBMotion.h"
#include "RBSharedMemory.h"
#include "RBMotorController.h"
#include "RBIMUSensor.h"
#include "RBCAN.h"
#include <sched.h>
#include <sys/types.h>
#include <libgen.h>
#include <stdio.h>
#include <dirent.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <stdarg.h>

#include "hubodog5_struct.h"
#include "BasicMath.h"
#include "BasicMatrix.h"
#include "func_kinematics.h"
#include "func_general.h"
#include "func_uiuk.h"
#include "qpSWIFT/include/Prime.h"
#include "RBJoystick.h"

#define READY   0
#define START   1
#define STOP    2
#define ONESTEP 3

extern pRBCORE_SHM sharedData;
class hubodog5_general
{

public:
    ////////////////////////////////////////////////////////////


    // COM control
    Vector3d com_control_force;

    Vector3d hip_height_force[4];

    RobotState state_motion;
    RobotState state_comp;
    RobotState state_out;

    int n_contact;
    int n_contact_pre;

    Vector3d foot_Force_COM[4];
    Vector3d foot_Force_FF[4];
    Vector3d foot_Force_ref[4];
    Vector3d foot_Torque_ref[4];

    int mode_state_task = 0;
    int mode_state_task_pre = 0;
    int joy_state = 0;
    int joy_state_task = 0;
    int joy_state_task_pre = 0;

    int mode_state = 0;
    int mode_state_pre = 0;
    int ready_flag = 0;
    int start_flag = 0;
    int stop_flag = 0;
    int end_flag = 0;
    int onestep_flag = 0;
    int step_num;
    int step_num_leg[4] = {0, };
    double t_step_leg[4] = {0, };

    double gain_change_time[4];
    int gain_change_cnt;

    bool leg_pos[4];

    bool contact_mode_flag;
    bool contact_flag[4], contact_flag_pre[4];
    bool contact_flag_test[4], contact_flag_test_pre[4];
    double contact_force[4];
    double landing_height[4];

    Vector3d tilt_est = Vector3dZero;

    bool landing_flag[4];

    bool PosUpdateflag = false;

    Vector3d Endpos_ST[4];
    Vector3d Endpos_SW[4];
    Vector3d Endvel_ST[4];
    Vector3d Endvel_SW[4];

    double t_temp;
    double t_temp_leg[4] = {0.,};

    Vector3d EndStep, DeltaStep;
    double DeltaStepGain = 0.5;

    double step_z;
    Vector3d pos_joy;
    Vector3d pos_joy_old;
    Vector3d vel_joy, vel_joy_filtered;
    Vector3d vel_joy_old;
    Vector3d vel_joy_oldstep;
    Vector3d acc_joy;
    Vector3d pos_step;
    Vector3d pos_step_old;
    double pos_rotate;

    double Joy_X = 0;
    double Joy_Y = 0;
    double Joy_R = 0;
    double Joy_X_pre = 0;
    double Joy_Y_pre = 0;
    double Joy_R_pre = 0;
    double Joy_X_acc = 0;
    double Joy_Y_acc = 0;
    double Joy_R_acc = 0;

    double Joy_Roll = 0;
    double Joy_Pitch = 0;
    double Joy_Yaw = 0;
    double Joy_Roll_pre = 0;
    double Joy_Pitch_pre = 0;
    double Joy_Yaw_pre = 0;

    Vector3d Joy_Orientation;
    Vector3d Joy_step;
    double Joy_rotation;

    bool Joy_flag;

    ////////////////////////////////////////////////////////////////
    //----------------- Global variables here -----------------
    double interval = 0.25;
    Vector3d Contact_point;
    Vector3d Global_COM;

    Vector3d Contact_point_ref;
    Vector3d Global_COM_ref;

    bool FirstStep = true;

    func_kinematics Kinematics;
    SensorInfo QuadSensor, QuadSensorRef, QuadSensor_start, QuadSensor_old, QuadSensor_filtered;
    RobotState QuadState, QuadStateRef, QuadState_start, QuadState_readypos, QuadState_old, QuadState_NOIMU;
    RobotState QuadStateIMU;
    PointState ComState, ComState_old;
    PointState ComStateRef, ComStateRef_old;

    Vector3d RampEulerRef = Vector3dZero;
    Vector4d RampQuatRef;
    Matrix3d RampMatRef;

    Vector3d ComLast_vel;

    double CoM_height_variation = 0;

    Matrix3d legJacobian[4], legJacobian_global[4], legJacobian_angleRef[4];
    bool isMotionEnabled, isMotionStopping;
    bool isSimulation;
    bool RaisimSimulation;
    int task_idx;
    bool Pcon[12], Ccon[12], PCcon[12];
    Vector3d Kp, Kd;
    Matrix3d Kp_mat, Kd_mat;
    QuadJoint TorqueRef;
    QuadJoint ComputedCurRef;
    QuadJoint CurRef; // current reference
    QuadJoint JointRef;
    int cnt_motion;
    double dt_main;
    double t_sim;
    bool isReadyDone;
    bool isPosAdjust;
    bool flag_contact[4], firstContact_ON[4];
    bool swing_phase[4];
    double z0, z0_start, x0, x0_start;
    QuadJoint PosAdjust_final, PosAdjust_init;
    //const double torque_const = 0.155 * 11.0;
    const double torque_const = 0.125 * 11.0;
    double current_limit_ = 20.0;

    // HB foot trajectory
    Vector3d x_dx_ddx[4], y_dy_ddy[4], z_dz_ddz[4];
    Vector3d x_dx_ddx_old[4], y_dy_ddy_old[4], z_dz_ddz_old[4];


    // save parameters
    const static int idx_JointPosEnc = 0;
    const static int idx_JointVel = idx_JointPosEnc+12;
    const static int idx_JointCurrent = idx_JointVel+12;
    const static int idx_JointPosEnc_filt = idx_JointCurrent + 12;
    const static int idx_JointVel_filt = idx_JointPosEnc_filt + 12;
    const static int idx_JointCurrent_filt = idx_JointVel_filt +12;
    const static int idx_IMUangle = idx_JointCurrent_filt+12; // Roll Pitch Yaw
    const static int idx_IMUquat = idx_IMUangle+3;
    const static int idx_IMUomega_raw = idx_IMUquat+4;
    const static int idx_IMUomega = idx_IMUomega_raw+3;
    const static int idx_IMUacc = idx_IMUomega+3;
    const static int idx_xmeasured = idx_IMUacc+3;
    const static int idx_xcurrent = idx_xmeasured+24;
    const static int idx_xref = idx_xcurrent+24;
    const static int idx_GRFRef = idx_xref+24;
    const static int idx_TorqueRef = idx_GRFRef+12;
    const static int idx_weight = idx_TorqueRef + 12;
    const static int idx_Nhorizon = idx_weight + 24+12+1;
    const static int idx_Tstep = idx_Nhorizon + 1;
    const static int idx_Ts_mpc = idx_Tstep + 1;
    const static int idx_mu = idx_Ts_mpc + 1;
    const static int idx_Fzmax = idx_mu + 1;
    const static int idx_mpcFrq_solve = idx_Fzmax + 1;
    const static int idx_mpcFrq_total = idx_mpcFrq_solve + 1;
    const static int idx_mpcIter = idx_mpcFrq_total + 1;
    const static int idx_contactflag = idx_mpcIter + 1;
    const static int idx_contactflag_ref = idx_contactflag + 4;
    const static int idx_extForce = idx_contactflag_ref + 4;

    const static int SaveMaxCnt = 1e5;
    const static int SaveMaxData = idx_extForce+12;
    double SAVE[SaveMaxData][SaveMaxCnt];


    void main_init();
    //----------------- Global functions here -----------------
    void Trot_Trajectory();

    void initialize();
    void motion_start();
    void motion_stop();
    void ALLPCON();
    void ALLCCON();
    void ALLPCCON();
    QuadJoint pos_adjust();
    void state_estimator();
    void MPC_calc();
    void save_onestep(int cnt);
    void save_file();

    //----------------- Task functions here -----------------
    QuadJoint task_demo();
    QuadJoint task_dance01();
    QuadJoint task_onelegtest();
    QuadJoint task_MPC();
    QuadJoint task_trot_test();
    QuadJoint task_flyingtrot();
    QuadJoint task_hopping_test();
    QuadJoint task_comcontrol_test();





    // -------------------- MPC variable ------------------------
    int Nhorizon;  // defaut = 20
    int m;         // # of state : [pb, dpb, gamma, omega, pf]
    int n;         // # of input : [RH - LH - RF - LF]
    int m2, n2;    // size of Aineq_x & Aineq_u
    int leg_num;
    int pf_num; // # of foot position number (xyz * 4 = 12)
    int cnt_MPC;
    VectorNd eta, eta_new; // boolean that indicates timing when new stepping occurs, MatrixNd::Zero(N,1)
    MatrixNd gait_boolean; // Matrix consist of all gait booleans during horizon
    VectorNd gait_current; // gait_boolean=MatrixNd::Zero(4,N), gait_current=MatrixNd::Zero(4,1)
    MatrixNd gait_sequence; int gait_N; // represent gait type from gait_library
    MatrixNd xref_vec, uref_vec, Rref_vec; // reference trajectory of state x, control u, rotation R
    Matrix3d R;
    VectorNd xcurrent, xop;
    VectorNd ucurrent, uop;
    VectorNd pfop;
    VectorNd x, xs, x_estimated, x_measured;
    Vector3d rpy_estimated, rpy_ref, rpy_old;
    int stepping;
    int swing_num;
    int total_stepping;
    double mu; // friction coefficient, default = 0.5
    double Fz_max;
    double ub_FootY; // only used when bound constraints on foot position exist.
    double Ts_mpc; // default = 0.02 (50Hz)
    double Tstep, T_sim, dt_to_Tmpc;
    int Nstep, gait_i;
    double maxiter, decaying_rate;
    double iter_num, mpc_flag, cost_value, Tsetup_mpc;
    MatrixNd Aop, Bop, Cop, Aineq_x, Aineq_u;
    MatrixNd Aeq, Aineq, Qmat;
    VectorNd bineq_x, bineq_u, beq, bineq, ys;
    void make_ABC(VectorNd _xop, VectorNd _uop, MatrixNd _Rop, VectorNd _pfop);
    void make_ineq(VectorNd _xop, MatrixNd _Rop, VectorNd _pfop);
    MatrixNd selection_mat_MPC(VectorNd boolean_prev, VectorNd boolean_next);
    void get_eta();
    Vector3d footref_CP(Vector3d vec_hip, Vector3d dpb, Vector3d dpb_s);
    void gait_gen();
    VectorNd find_index(VectorNd input_vec, int value);
    VectorNd gravity;
    MatrixNd Nmat, Nmat_inv;
    Matrix3d inertia_mat, Imat_inverse;
    MatrixNd eye_mat;
    Vector3d nx_RH, ny_RH, nz_RH, nx_LH, ny_LH, nz_LH, nx_RF, ny_RF, nz_RF, nx_LF, ny_LF, nz_LF;
    MatrixNd nvec_mat;
    double UB_pfx, UB_pfy; // box constraints of foot position
    Vector3d proj_x, proj_y;
    MatrixNd proj_mat;
    VectorNd weight_mpc, weight_mpc_stand, weight_mpc_trot, weight_mpc_trot2, weight_mpc_crawl;
    struct gait_library
    {
        MatrixNd trot;
        MatrixNd trot2;
        MatrixNd stand;
        MatrixNd crawl;
    };
    gait_library gait_library;
    std::string TaskName, CustomFileName;

    void get_Air(MatrixNd mat, qp_int* Air);
    void get_Ajc(MatrixNd mat, qp_int* Ajc);
    void get_Apr(MatrixNd mat, qp_real* Apr);
    void vec_to_realtype(VectorNd vec, qp_real* vec_pnt);
    VectorNd qpSwift_MPC();
    double mpc_gain;
    bool isMPCON, isStopByUser, isLPFON_MPC, isSavingData;
    double LPF_frq_MPC;
    VectorNd Qvec;
    MatrixNd gait_sequence_next;
    bool gait_changing_flag;
    int gait_changing_idx;
    bool isSamplingLock, isMPCLock, isMPCstarted;
    VectorNd uref, dpf_new;

    // MPC & qpSWIFT variables
    Vector3d pbop;
    Vector3d wop;
    Vector3d uop_sum;
    MatrixNd uop_mat;
    MatrixNd r_mat;
    Vector3d uop_temp;
    Vector3d pf_temp;
    Vector3d r_temp;
    VectorNd torque;
    Matrix3d wop_hat;
    Matrix3d uop_sum_hat;
    Matrix3d inertia_wop_hat;
    MatrixNd kron_eye_Rop;
    MatrixNd kron_eye_RopT;
    VectorNd C1_0;
    MatrixNd C1_gamma;
    MatrixNd C1_w;
    VectorNd C2_0;
    MatrixNd C2_pb;
    MatrixNd C2_gamma;
    MatrixNd C2_w;
    MatrixNd C2_u;
    MatrixNd C2_pf;
    VectorNd mpc_result;
    double ys_size;
    int lock_size = 1e10;

//    QPswift* myQP;
    qp_int *Pjc;
    qp_int *Pir;
    qp_real *Ppr;
    qp_int *Ajc;
    qp_int *Air;
    qp_real *Apr;
    qp_int *Gjc;
    qp_int *Gir;
    qp_real *Gpr;
    qp_real *qpswift_c;
    qp_real *qpswift_h;
    qp_real *qpswift_b;
    qp_real sigma_d;
    qp_int optvar_size;
    qp_int ineq_size;
    qp_int eq_size;
    MatrixNd H;
    VectorNd F;
    void CCS_CLEANUP();


    //QP Walking
    QuadJoint QP_Standing();






};

//void reset_trot_controller(){

//}

#endif // HUBODOG5_GENERAL_H
