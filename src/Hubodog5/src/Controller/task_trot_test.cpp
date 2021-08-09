#include "hubodog5_general.h"
#include "func_sensor.h"
#include "RBSharedMemory.h"
#include "ESTIMATOR/estimator.h"
#include "ESTIMATOR_MIT/PositionVelocityEstimator.h"

#define SW  0
#define ST  1

#define TROT_WALKING        0
#define F_TROT_WALKING      1
#define TROT_WALKING_S      2
#define PRONKING            3


extern RBMotorController   _DEV_MC[MAX_MC];
extern int MY_CONTROL_FB_GAIN[12];
extern int MY_CONTROL_FF_GAIN[12];

//////////////////////////////////////////////////////



Vector3d com_foot_step, com_foot_step_filtered;
Vector3d com_foot_step_joy;
Vector3d com_foot_step_old;
Vector3d com_foot_step_hpf;
Vector3d com_foot_step_lpf;
Vector3d com_foot_step_bpf;
Vector3d imu_angle_hpf;

Vector3d joy_acc_modi_step = Vector3dZero;
Vector3d dist_modi_step = Vector3dZero;

int trot_sinit_flag = true;
int trot_init_flag = true;
int step_init_flag = false;

int trot_mode = 0;
int trot_mode_pre = 0;

double Joy_ComZ = 0;
Vector3d Ramp_radian_Ref;

Vector3d zmp_offset, zmp_offset_pre;

double w = sqrt(9.8/0.4);

int joy_zero_flag = 0;
double joy_sum = 0;



Estimator estimator(RT_TIMER_PERIOD_MS / 1000);
PositionVelocityEstimator estimator2;

QuadJoint hubodog5_general::task_trot_test()
{


//    interval = sharedData->trot_variable_double[0];
    double interval_pause = sharedData->trot_variable_double[1];
    Vector3d ff_dxy = Vector3d(sharedData->trot_variable_double[2], sharedData->trot_variable_double[3], 0);
    double ff_dw = sharedData->trot_variable_double[4];
    double ff_rcomp = sharedData->trot_variable_double[5]*D2Rf;
    double contact_threshold = sharedData->trot_variable_double[6];
    Vector3d contact_el_offset;

    int num_motion = 4;
    double timeline[num_motion*2+2]; timeline[0] = 0.0;

    double rotate_comp;

    double contact_checktime = 0.75;



    //interval = sharedData->Walking_period;

//    cout << "interval : " << interval << endl;

//    if(trot_mode == F_TROT_WALKING) {
//        interval = 0.25;
//    }


    contact_mode_flag = sharedData->trot_variable_int[1];

    mode_state_task_pre = mode_state_task;
    mode_state_task = sharedData->trot_variable_int[0];

    //cout << "mode task : " << sharedData->trot_variable_int[0] << endl;
    if(mode_state_task != mode_state_task_pre) {
        if(mode_state_task == START){
            if(joy_sum < 0.001)
                mode_state = START;
        }
        else{
            mode_state = mode_state_task;
        }
    }


//    joy_state_task_pre = joy_state_task;
//    joy_state_task = sharedData->joy_state;
//    if(joy_state_task != joy_state_task_pre) mode_state = joy_state_task;

    ready_flag = false;
    start_flag = false;
    stop_flag = false;
    onestep_flag = false;

    landing_flag[HR] = false;
    landing_flag[FL] = false;
    landing_flag[HL] = false;
    landing_flag[FR] = false;

    mode_state_pre = mode_state;

    if(mode_state == READY)
        ready_flag = true;
    else if(mode_state == START)
        start_flag = true;
    else if(mode_state == STOP)
        stop_flag = true;
    else if(mode_state == ONESTEP)
        onestep_flag = true;


    int kp_r = sharedData->Kp_j[0];
    int kp_p = sharedData->Kp_j[1];
    int kp_k = sharedData->Kp_j[2];
    int kd_r = sharedData->Kd_j[0];
    int kd_p = sharedData->Kd_j[1];
    int kd_k = sharedData->Kd_j[2];

    // start init
    if(trot_sinit_flag == true) {
        trot_sinit_flag = false;

        cout << "start init, variable initialize" << endl;


        sharedData->Walking_period = 0.25;
        sharedData->RaisimGainState = true;

        if(sharedData->REF_Enabled == true) {

            if(sharedData->task_legcon[HR]) {
                _DEV_MC[HRR].RBFOC_SetGain_POS(kp_r,0,kd_r, 1);
                _DEV_MC[HRP].RBFOC_SetGain_POS(kp_p,0,kd_p, 1);
                _DEV_MC[HRK].RBFOC_SetGain_POS(kp_k,0,kd_k, 1);
            }
            if(sharedData->task_legcon[HL]) {
                _DEV_MC[HLR].RBFOC_SetGain_POS(kp_r,0,kd_r, 1);
                _DEV_MC[HLP].RBFOC_SetGain_POS(kp_p,0,kd_p, 1);
                _DEV_MC[HLK].RBFOC_SetGain_POS(kp_k,0,kd_k, 1);
            }
            if(sharedData->task_legcon[FR]) {
                _DEV_MC[FRR].RBFOC_SetGain_POS(kp_r,0,kd_r, 1);
                _DEV_MC[FRP].RBFOC_SetGain_POS(kp_p,0,kd_p, 1);
                _DEV_MC[FRK].RBFOC_SetGain_POS(kp_k,0,kd_k, 1);
            }
            if(sharedData->task_legcon[FL]) {
                _DEV_MC[FLR].RBFOC_SetGain_POS(kp_r,0,kd_r, 1);
                _DEV_MC[FLP].RBFOC_SetGain_POS(kp_p,0,kd_p, 1);
                _DEV_MC[FLK].RBFOC_SetGain_POS(kp_k,0,kd_k, 1);
            }
        }

        state_motion = QuadState_start;

        Endpos_ST[HR] = QuadState_start.foot.HR.pos;
        Endpos_ST[FL] = QuadState_start.foot.FL.pos;
        Endpos_ST[HL] = QuadState_start.foot.HL.pos;
        Endpos_ST[FR] = QuadState_start.foot.FR.pos;

        Endpos_SW[HR] = QuadState_start.foot.HR.pos;
        Endpos_SW[FL] = QuadState_start.foot.FL.pos;
        Endpos_SW[HL] = QuadState_start.foot.HL.pos;
        Endpos_SW[FR] = QuadState_start.foot.FR.pos;

        x_dx_ddx[HR] = Vector3d(QuadState_start.foot.HR.pos.x(),0,0);
        x_dx_ddx[FL] = Vector3d(QuadState_start.foot.FL.pos.x(),0,0);
        x_dx_ddx[HL] = Vector3d(QuadState_start.foot.HL.pos.x(),0,0);
        x_dx_ddx[FR] = Vector3d(QuadState_start.foot.FR.pos.x(),0,0);

        y_dy_ddy[HR] = Vector3d(QuadState_start.foot.HR.pos.y(),0,0);
        y_dy_ddy[FL] = Vector3d(QuadState_start.foot.FL.pos.y(),0,0);
        y_dy_ddy[HL] = Vector3d(QuadState_start.foot.HL.pos.y(),0,0);
        y_dy_ddy[FR] = Vector3d(QuadState_start.foot.FR.pos.y(),0,0);

        z_dz_ddz[HR] = Vector3d(QuadState_start.foot.HR.pos.z(),0,0);
        z_dz_ddz[FL] = Vector3d(QuadState_start.foot.FL.pos.z(),0,0);
        z_dz_ddz[HL] = Vector3d(QuadState_start.foot.HL.pos.z(),0,0);
        z_dz_ddz[FR] = Vector3d(QuadState_start.foot.FR.pos.z(),0,0);

        x_dx_ddx_old[HR] = Vector3d(QuadState_start.foot.HR.pos.x(),0,0);
        x_dx_ddx_old[FL] = Vector3d(QuadState_start.foot.FL.pos.x(),0,0);
        x_dx_ddx_old[HL] = Vector3d(QuadState_start.foot.HL.pos.x(),0,0);
        x_dx_ddx_old[FR] = Vector3d(QuadState_start.foot.FR.pos.x(),0,0);

        y_dy_ddy_old[HR] = Vector3d(QuadState_start.foot.HR.pos.y(),0,0);
        y_dy_ddy_old[FL] = Vector3d(QuadState_start.foot.FL.pos.y(),0,0);
        y_dy_ddy_old[HL] = Vector3d(QuadState_start.foot.HL.pos.y(),0,0);
        y_dy_ddy_old[FR] = Vector3d(QuadState_start.foot.FR.pos.y(),0,0);

        z_dz_ddz_old[HR] = Vector3d(QuadState_start.foot.HR.pos.z(),0,0);
        z_dz_ddz_old[FL] = Vector3d(QuadState_start.foot.FL.pos.z(),0,0);
        z_dz_ddz_old[HL] = Vector3d(QuadState_start.foot.HL.pos.z(),0,0);
        z_dz_ddz_old[FR] = Vector3d(QuadState_start.foot.FR.pos.z(),0,0);

        joy_state_task = 0;
        mode_state_task = 0;

        com_foot_step_filtered = Vector3dZero;
        com_foot_step = Vector3dZero;
        imu_angle_hpf = Vector3dZero;
        com_foot_step_bpf = Vector3dZero;

        for(int i=0; i<4; i++){
            foot_Force_FF[i] = Vector3dZero;
            foot_Force_COM[i] = Vector3dZero;
            foot_Force_ref[i] = Vector3dZero;
        }
        com_control_force = Vector3dZero;
        vel_joy = Vector3dZero;
        acc_joy = Vector3dZero;

        ComState.vel = Vector3dZero;
        ComStateRef.vel = Vector3dZero;
        ComStateRef_old.pos = Vector3dZero;
        ComState_old.pos = Vector3dZero;
        ComStateRef.pos = Vector3dZero;
        ComState.pos = Vector3dZero;

        Contact_point = ComState.pos;
        Contact_point_ref = ComState.pos;

        FirstStep = true;
        ready_flag = true;
        start_flag = false;
        stop_flag = false;
        onestep_flag = false;
        mode_state = 0;

        pos_step = Vector3dZero;

        zmp_offset = Vector3dZero;
        zmp_offset_pre = Vector3dZero;
        dist_modi_step = Vector3dZero;
        joy_acc_modi_step = Vector3dZero;
        RampEulerRef = Vector3dZero;
        tilt_est = Vector3dZero;

        //estimator init
        Eigen::Vector3d init_pos(0, 0, 0.28);
        Eigen::VectorXd init_joint(12);

        init_joint << QuadSensor.Encoder.pos.HRR, QuadSensor.Encoder.pos.HRP, QuadSensor.Encoder.pos.HRK,
                      QuadSensor.Encoder.pos.HLR, QuadSensor.Encoder.pos.HLP, QuadSensor.Encoder.pos.HLK,
                      QuadSensor.Encoder.pos.FRR, QuadSensor.Encoder.pos.FRP, QuadSensor.Encoder.pos.FRK,
                      QuadSensor.Encoder.pos.FLR, QuadSensor.Encoder.pos.FLP, QuadSensor.Encoder.pos.FLK;

        estimator.Init(init_pos, init_joint, QuadSensor.IMU.quat);

        //estimator2 init
        estimator2.setup(dt_main);


        cout << "trot start init" << endl;
    }
    // Estimator
    Eigen::Matrix3d body_rot;
    Operation::quatToRotation(QuadSensor.IMU.quat, body_rot);

    Eigen::VectorXd joint_pos(12), joint_vel(12), joint_input(12);

    Eigen::Vector4i contact(4); contact<< contact_flag_test[0], contact_flag_test[1], contact_flag_test[2], contact_flag_test[3];

    joint_pos << QuadSensor.Encoder.pos.HRR, QuadSensor.Encoder.pos.HRP, QuadSensor.Encoder.pos.HRK,
                 QuadSensor.Encoder.pos.HLR, QuadSensor.Encoder.pos.HLP, QuadSensor.Encoder.pos.HLK,
                 QuadSensor.Encoder.pos.FRR, QuadSensor.Encoder.pos.FRP, QuadSensor.Encoder.pos.FRK,
                 QuadSensor.Encoder.pos.FLR, QuadSensor.Encoder.pos.FLP, QuadSensor.Encoder.pos.FLK;

    joint_vel << QuadSensor.Encoder.vel.HRR, QuadSensor.Encoder.vel.HRP, QuadSensor.Encoder.vel.HRK,
                 QuadSensor.Encoder.vel.HLR, QuadSensor.Encoder.vel.HLP, QuadSensor.Encoder.vel.HLK,
                 QuadSensor.Encoder.vel.FRR, QuadSensor.Encoder.vel.FRP, QuadSensor.Encoder.vel.FRK,
                 QuadSensor.Encoder.vel.FLR, QuadSensor.Encoder.vel.FLP, QuadSensor.Encoder.vel.FLK;




//    joint_vel << 0, 0, 0,
//                 0, 0, 0,
//                 0, 0, 0,
//                 0, 0, 0;




    Eigen::Vector3d w_pos_body, w_vel_body;

    estimator.estimate2(body_rot,QuadSensor.IMU.acc, QuadSensor.IMU.vel, contact, joint_pos, joint_vel, w_pos_body, w_vel_body);

    static Vector3d w_pos_old;// = Vector3dZero;

    Vector3d vel_pos = (w_pos_body - w_pos_old)/dt_main;

    w_pos_old = w_pos_body;

    static Vector3d w_vel_filtered = Vector3dZero;

    w_vel_filtered = Lowpass_filter_3d(w_vel_body, w_vel_filtered, 1.0, dt_main);

    // Estimator MIT
    Eigen::Vector4d contactd(4); contact<< contact_flag[0], contact_flag[1], contact_flag[2], contact_flag[3];
    //Eigen::Vector4d contactd(4); contact<< 1,1,1,1;
    estimator2.estimate(QuadSensor.IMU.quat, QuadSensor.IMU.acc, QuadSensor.IMU.vel, contactd, QuadState_NOIMU);


    //
    if(trot_init_flag == true) {

        if(start_flag == true) {
            cout << "Trot Test" << endl;
            TaskName = "Trot";
            step_init_flag = true;
        }

        trot_init_flag = false;
        for(int i=0; i<4; i++) {
            contact_flag[i] = true;
        }

        cout << "trot init" << endl;
    }

    // init every end of cycle
    if(step_init_flag == true) {
        step_init_flag = false;
        trot_mode = sharedData->trot_variable_int[2];
        if(sharedData->Walking_period == 0.15){
            interval = 0.15;
        }
        else if(sharedData->Walking_period == 0.2){
            interval = 0.2;
        }
        else if(sharedData->Walking_period == 0.25){
            interval = 0.25;
        }
        else if(sharedData->Walking_period == 0.3){
            interval = 0.3;
        }
        //FirstStep = false;
    }

//    for(int i=0; i<num_motion; i++) {
//        timeline[i*2+1] = timeline[i*2] + interval;
//        timeline[i*2+2] = timeline[i*2+1] + interval_pause;
//    }

    timeline[0] = 0;
    timeline[1] = 0.3;
    timeline[2] = timeline[1] + interval_pause;
    timeline[3] = timeline[2] + interval;
    timeline[4] = timeline[3] + interval_pause;
    timeline[5] = timeline[4] + interval;
    timeline[6] = timeline[5] + interval_pause;
    timeline[7] = timeline[6] + 0.3;
    timeline[8] = timeline[7] + interval_pause;


    // trot mode selection
    trot_mode_pre = trot_mode;
    if(step_num == -1) trot_mode = sharedData->trot_variable_int[2];

    if(trot_mode != trot_mode_pre) {
        if(trot_mode == F_TROT_WALKING) {
            if(sharedData->REF_Enabled == true) {

                int kp_f = 10;
                if(sharedData->task_legcon[HR]) {
                    _DEV_MC[HRR].RBFOC_SetGain_POS(kp_f,0,kd_r, 1);
                    _DEV_MC[HRP].RBFOC_SetGain_POS(kp_f,0,kd_p, 1);
                    _DEV_MC[HRK].RBFOC_SetGain_POS(kp_f,0,kd_k, 1);
                }
                if(sharedData->task_legcon[HL]) {
                    _DEV_MC[HLR].RBFOC_SetGain_POS(kp_f,0,kd_r, 1);
                    _DEV_MC[HLP].RBFOC_SetGain_POS(kp_f,0,kd_p, 1);
                    _DEV_MC[HLK].RBFOC_SetGain_POS(kp_f,0,kd_k, 1);
                }
                if(sharedData->task_legcon[FR]) {
                    _DEV_MC[FRR].RBFOC_SetGain_POS(kp_f,0,kd_r, 1);
                    _DEV_MC[FRP].RBFOC_SetGain_POS(kp_f,0,kd_p, 1);
                    _DEV_MC[FRK].RBFOC_SetGain_POS(kp_f,0,kd_k, 1);
                }
                if(sharedData->task_legcon[FL]) {
                    _DEV_MC[FLR].RBFOC_SetGain_POS(kp_f,0,kd_r, 1);
                    _DEV_MC[FLP].RBFOC_SetGain_POS(kp_f,0,kd_p, 1);
                    _DEV_MC[FLK].RBFOC_SetGain_POS(kp_f,0,kd_k, 1);
                }
            }
        }
        else {
            if(sharedData->REF_Enabled == true) {

                if(sharedData->task_legcon[HR]) {
                    _DEV_MC[HRR].RBFOC_SetGain_POS(kp_r,0,kd_r, 1);
                    _DEV_MC[HRP].RBFOC_SetGain_POS(kp_p,0,kd_p, 1);
                    _DEV_MC[HRK].RBFOC_SetGain_POS(kp_k,0,kd_k, 1);
                }
                if(sharedData->task_legcon[HL]) {
                    _DEV_MC[HLR].RBFOC_SetGain_POS(kp_r,0,kd_r, 1);
                    _DEV_MC[HLP].RBFOC_SetGain_POS(kp_p,0,kd_p, 1);
                    _DEV_MC[HLK].RBFOC_SetGain_POS(kp_k,0,kd_k, 1);
                }
                if(sharedData->task_legcon[FR]) {
                    _DEV_MC[FRR].RBFOC_SetGain_POS(kp_r,0,kd_r, 1);
                    _DEV_MC[FRP].RBFOC_SetGain_POS(kp_p,0,kd_p, 1);
                    _DEV_MC[FRK].RBFOC_SetGain_POS(kp_k,0,kd_k, 1);
                }
                if(sharedData->task_legcon[FL]) {
                    _DEV_MC[FLR].RBFOC_SetGain_POS(kp_r,0,kd_r, 1);
                    _DEV_MC[FLP].RBFOC_SetGain_POS(kp_p,0,kd_p, 1);
                    _DEV_MC[FLK].RBFOC_SetGain_POS(kp_k,0,kd_k, 1);
                }
            }
        }
    }

    if(trot_mode == TROT_WALKING) {
        contact_el_offset = Vector3d(0,0,sharedData->trot_variable_double[7]);
        step_z = sharedData->trot_variable_double[10];
    }
    else if(trot_mode == F_TROT_WALKING) {
        contact_el_offset = Vector3d(0, 0, 0.04);
        step_z = 0.12;
    }
    else if(trot_mode == TROT_WALKING_S) {
        contact_el_offset = Vector3d(0, 0, sharedData->trot_variable_double[7]);
        step_z = sharedData->trot_variable_double[10];
    }


    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Joystick

    Joy_X_pre = Joy_X;
    Joy_Y_pre = Joy_Y;
    Joy_R_pre = Joy_R;
    Joy_Roll_pre = Joy_Roll;
    Joy_Pitch_pre = Joy_Pitch;
    Joy_Yaw_pre = Joy_Yaw;

    Joy_X = Lowpass_filter(sharedData->joy_dpCOM[0], Joy_X, 1.0, dt_main); // 5.0
    Joy_Y = Lowpass_filter(sharedData->joy_dpCOM[1], Joy_Y, 1.0, dt_main);
    Joy_R =  Lowpass_filter(sharedData->joy_dpCOM[3], Joy_R, 1.0, dt_main);

    double f_c = 0.5;
    Joy_Roll = Lowpass_filter(-sharedData->joy_dRPY[0], Joy_Roll, f_c, dt_main);
    Joy_Pitch = Lowpass_filter(-sharedData->joy_dRPY[1], Joy_Pitch, f_c, dt_main);
    Joy_Yaw =  Lowpass_filter(sharedData->joy_dRPY[2], Joy_Yaw, f_c, dt_main);

    Joy_X_acc = (Joy_X - Joy_X_pre)/dt_main;
    Joy_Y_acc = (Joy_Y - Joy_Y_pre)/dt_main;
    Joy_R_acc = (Joy_R - Joy_R_pre)/dt_main;

//    Ramp_radian_Ref.x() = 0.;
//    Ramp_radian_Ref.z() = 0.;
//    Ramp_radian_Ref.y() = Lowpass_filter(-sharedData->joy_dpCOM[2], Ramp_radian_Ref.y(), 1, dt_main);
//    RampEulerRef = Ramp_radian_Ref;

//    RampEulerRef = Vector3d(0, tilt_est.y(), 0);
    RampEulerRef = Vector3d(tilt_est.x(), tilt_est.y(), 0);
//    RampEulerRef = Vector3dZero;

    if(start_flag == true) {
        Joy_step = Vector3d(Joy_X, Joy_Y, 0);
        Joy_rotation = -Joy_R;

        Joy_Roll = 0;
        Joy_Pitch = 0;
        Joy_Yaw = 0;
        Joy_Roll_pre = 0;
        Joy_Pitch_pre = 0;
        Joy_Yaw_pre = 0;
        Joy_Orientation = Vector3dZero;

        joy_zero_flag = false;

    }
    else {
        joy_sum = (Joy_Roll*Joy_Roll) + (Joy_Pitch*Joy_Pitch) + (Joy_Yaw*Joy_Yaw);
        if(joy_sum < 0.000001) joy_zero_flag = true;

        if(joy_zero_flag == true) {
            Joy_Orientation = Vector3d(Joy_Roll*0.3, Joy_Pitch*0.3, Joy_Yaw*0.4);
        }
    }

    sharedData->joy_Orientation[0] = Joy_Orientation.x();
    sharedData->joy_Orientation[1] = Joy_Orientation.y();
    sharedData->joy_Orientation[2] = Joy_Orientation.z();

 //    cout << "sum : " << joy_sum[0] << ", flag : " << joy_zero_flag[0] << ",   sum : " << joy_sum[1] << ", flag : " << joy_zero_flag[1] << endl;


    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Stepping

    Vector3d com_Kp_p = Vector3d(sharedData->trot_variable_double[11] ,sharedData->trot_variable_double[13], sharedData->trot_variable_double[15]);
    Vector3d com_Kd_p = Vector3d(sharedData->trot_variable_double[12] ,sharedData->trot_variable_double[14], sharedData->trot_variable_double[16]);

    if(trot_mode == 1) {
        com_Kp_p = Vector3d(0,0,0);
        com_Kd_p = Vector3d(0,0,0);
    }

    Matrix3d com_Kp_mat_p = diagonalize(com_Kp_p);
    Matrix3d com_Kd_mat_p = diagonalize(com_Kd_p);
    double hpf_frq = sharedData->control_variable_double[0];

    imu_angle_hpf.x() = Highpass_filter(QuadSensor.IMU.vel.x()*dt_main, 0, imu_angle_hpf.x(),hpf_frq,dt_main);
    imu_angle_hpf.y() = Highpass_filter(QuadSensor.IMU.vel.y()*dt_main, 0, imu_angle_hpf.y(),hpf_frq,dt_main);
    imu_angle_hpf.z() = Highpass_filter(QuadSensor.IMU.vel.z()*dt_main, 0, imu_angle_hpf.z(),hpf_frq,dt_main);

    Vector3d imu_angle_hpf_temp = -(imu_angle_hpf);

    //com_foot_step_hpf = com_Kp_mat_p*-imu_angle_hpf;

    double angle_limit = 2.0*D2Rf;
    Vector3d imu_angle_hpf_l;
    for(int i=0; i<3; i++) {
        if(imu_angle_hpf_temp[i] > angle_limit) {
            imu_angle_hpf_l[i] = imu_angle_hpf_temp[i] - angle_limit;
        }
        else if(imu_angle_hpf_temp[i] < -angle_limit) {
            imu_angle_hpf_l[i] = imu_angle_hpf_temp[i] + angle_limit;
        }
        else imu_angle_hpf_l[i] = 0;
    }

    com_foot_step_hpf = com_Kp_mat_p*imu_angle_hpf_temp; // check plz...

    com_foot_step_lpf.x() = Lowpass_filter(com_foot_step.x(), com_foot_step_lpf.x(), 10, dt_main);
    com_foot_step_lpf.y() = Lowpass_filter(com_foot_step.y(), com_foot_step_lpf.y(), 10, dt_main);
    com_foot_step_lpf.z() = Lowpass_filter(com_foot_step.z(), com_foot_step_lpf.z(), 10, dt_main);

    com_foot_step_bpf.x() = Lowpass_filter(com_foot_step_hpf.x(), com_foot_step_bpf.x(), 0.2, dt_main);
    com_foot_step_bpf.y() = Lowpass_filter(com_foot_step_hpf.y(), com_foot_step_bpf.y(), 0.2, dt_main);
    com_foot_step_bpf.z() = Lowpass_filter(com_foot_step_hpf.z(), com_foot_step_bpf.z(), 0.2, dt_main);

    double limit_p[3] = {0.1, 0.1, 20*D2Rf};
    for(int i=0; i<3; i++) {
        if(com_foot_step_lpf[i] > limit_p[i]) com_foot_step_lpf[i] = limit_p[i];
        else if(com_foot_step_lpf[i] < -limit_p[i]) com_foot_step_lpf[i] = -limit_p[i];
    }
    for(int i=0; i<3; i++) {
        if(com_foot_step_bpf[i] > limit_p[i]) com_foot_step_bpf[i] = limit_p[i];
        else if(com_foot_step_bpf[i] < -limit_p[i]) com_foot_step_bpf[i] = -limit_p[i];
    }


    pos_joy = Joy_step + ff_dxy;
    vel_joy_oldstep = vel_joy;
    vel_joy = -pos_joy/interval;
    acc_joy = Lowpass_filter_3d((vel_joy - vel_joy_old)/dt_main,acc_joy, 1, dt_main);
    vel_joy_old = vel_joy;
    pos_rotate = Joy_rotation + ff_dw;
    vel_joy_filtered = Lowpass_filter_3d(-vel_joy, vel_joy_filtered, 1, dt_main);

    double x_temp = sharedData->trot_variable_double[17];
    double y_temp = sharedData->trot_variable_double[18];
    double z_temp = sharedData->trot_variable_double[19];

    double x_dist_stepping = sharedData->trot_variable_double[25];
    double y_dist_stepping = sharedData->trot_variable_double[26];

    if(trot_mode == F_TROT_WALKING) {
        x_temp = 0;
        y_temp = 0;
        z_temp = 0;

        x_dist_stepping = 0;
        y_dist_stepping = 0;
    }


    ////------------------- Ref acc from Joystick command
    //ref centripetal force
    double radius_of_curvature = 0;
    if(tan(pos_rotate/2.0) != 0){
        radius_of_curvature = 0.5*vel_joy.x()/(tan(pos_rotate/interval/2.0));
    }

    Vector3d CoM_acc_FF = Vector3dZero;
    CoM_acc_FF.x() = acc_joy.x();

    if(radius_of_curvature != 0){
//        CoM_acc_FF.y() = acc_joy.y() - 0.7*vel_joy.x()*vel_joy.x()/radius_of_curvature;
        CoM_acc_FF.y() = -vel_joy.x()*vel_joy.x()/radius_of_curvature;
    }
    else{
        CoM_acc_FF.y() = acc_joy.y();
    }

    if(FirstStep == false ) {

        joy_acc_modi_step.x() = CoM_acc_FF.x()*x_temp;
        joy_acc_modi_step.y() = CoM_acc_FF.y()*y_temp;

        com_foot_step.x() = com_Kd_p.x()*(ComState.vel.x() - ComStateRef.vel.x()) ;
        com_foot_step.y() = com_Kd_p.y()*(ComState.vel.y() - ComStateRef.vel.y()) ;

        double limit_p[2] = {0.15, 0.15};
        for(int i=0; i<2; i++) {
            if(com_foot_step[i] > limit_p[i]) com_foot_step[i] = limit_p[i];
            else if(com_foot_step[i] < -limit_p[i]) com_foot_step[i] = -limit_p[i];
        }

        dist_modi_step.x() = x_dist_stepping*(ComState.vel.x() - (-vel_joy.x())) ;
        dist_modi_step.y() = y_dist_stepping*(ComState.vel.y() - (-vel_joy.y())) ;

        //printf("stepx:%.3f, comvelx:%.3f, refvelx:%.3f\n", com_foot_step.y(), ComState.vel.y(), ComStateRef.vel.y());

    }
    else {
        com_foot_step_bpf = Vector3dZero;
        com_foot_step = Vector3dZero;
        com_foot_step_filtered = Vector3dZero;
        imu_angle_hpf = Vector3dZero;
    }

    pos_step = Vector3d(com_foot_step.x()-com_foot_step_bpf.y(), com_foot_step.y() + com_foot_step_bpf.x(), 0) + pos_joy;
    pos_step.z() = 0.0;

    for(int i=0; i<4; i++){
        contact_flag_pre[i] = contact_flag[i];
        contact_flag_test_pre[i] = contact_flag_test[i];
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Timeline

    if(t_sim < 0)                                    step_num = -1;
    else if(t_sim<timeline[1])                       step_num = 0;
    else if(t_sim>=timeline[1] && t_sim<timeline[2]) step_num = 1;
    else if(t_sim>=timeline[2] && t_sim<timeline[3]) step_num = 2;
    else if(t_sim>=timeline[3] && t_sim<timeline[4]) step_num = 3;
    else if(t_sim>=timeline[4] && t_sim<timeline[5]) step_num = 4;
    else if(t_sim>=timeline[5] && t_sim<timeline[6]) step_num = 5;
    else if(t_sim>=timeline[6] && t_sim<timeline[7]) step_num = 6;
    else if(t_sim>=timeline[7] && t_sim<timeline[8]) step_num = 7;
    else                                             step_num = 8;

    if(step_num == -1) {

    }

    else if(step_num == 0){ // ------------------ motion 01 --------------------- start motion
        t_temp = t_sim-timeline[0];

        if(t_temp >= contact_checktime*interval) {
            landing_flag[HR] = true;
            landing_flag[FL] = true;
        }

        // HB version
        //HR-stance
        Vector3d pos_start_HR = QuadState_start.foot.HR.pos;
        Vector3d pos_end_HR = QuadState_start.foot.HR.pos;
        x_dx_ddx[HR] = FootXY_trajectory(t_temp, interval-dt_main, pos_start_HR.x(), pos_end_HR.x(), 0, x_dx_ddx_old[HR]);
        y_dy_ddy[HR] = FootXY_trajectory(t_temp, interval-dt_main, pos_start_HR.y(), pos_end_HR.y(), 0, y_dy_ddy_old[HR]);
        z_dz_ddz[HR] = FootXY_trajectory(t_temp, interval-dt_main, pos_start_HR.z(), pos_end_HR.z(), 0, z_dz_ddz_old[HR]);
        state_motion.foot.HR.pos = Vector3d(x_dx_ddx[HR].x(), y_dy_ddy[HR].x(), z_dz_ddz[HR].x());
        state_motion.foot.HR.vel = Vector3d(x_dx_ddx[HR].y(), y_dy_ddy[HR].y(), z_dz_ddz[HR].y());

        //FL-stance
        Vector3d pos_start_FL = QuadState_start.foot.FL.pos;
        Vector3d pos_end_FL = QuadState_start.foot.FL.pos;
        x_dx_ddx[FL] = FootXY_trajectory(t_temp, interval-dt_main,pos_start_FL.x(), pos_end_FL.x(), 0, x_dx_ddx_old[FL]);
        y_dy_ddy[FL] = FootXY_trajectory(t_temp, interval-dt_main,pos_start_FL.y(), pos_end_FL.y(), 0, y_dy_ddy_old[FL]);
        z_dz_ddz[FL] = FootXY_trajectory(t_temp, interval-dt_main,pos_start_FL.z(), pos_end_FL.z(), 0, z_dz_ddz_old[FL]);
        state_motion.foot.FL.pos = Vector3d(x_dx_ddx[FL].x(), y_dy_ddy[FL].x(), z_dz_ddz[FL].x());
        state_motion.foot.FL.vel = Vector3d(x_dx_ddx[FL].y(), y_dy_ddy[FL].y(), z_dz_ddz[FL].y());

        //HL-swing
        Vector3d pos_start_HL = QuadState_start.foot.HL.pos;
        Vector3d pos_end_HL = QuadState_start.foot.HL.pos;
        x_dx_ddx[HL] = FootXY_trajectory(t_temp, interval-dt_main,pos_start_HL.x(), pos_end_HL.x(), 0, x_dx_ddx_old[HL]);
        y_dy_ddy[HL] = FootXY_trajectory(t_temp, interval-dt_main,pos_start_HL.y(), pos_end_HL.y(), 0, y_dy_ddy_old[HL]);
        z_dz_ddz[HL] = FootZ_Swing_trajectory(t_temp, interval-dt_main,pos_start_HL.z(), pos_end_HL.z(), step_z, z_dz_ddz_old[HL]);
        state_motion.foot.HL.pos = Vector3d(x_dx_ddx[HL].x(), y_dy_ddy[HL].x(), z_dz_ddz[HL].x());
        state_motion.foot.HL.vel = Vector3d(x_dx_ddx[HL].y(), y_dy_ddy[HL].y(), z_dz_ddz[HL].y());

        //FR-swing
        Vector3d pos_start_FR = QuadState_start.foot.FR.pos;
        Vector3d pos_end_FR = QuadState_start.foot.FR.pos;
        x_dx_ddx[FR] = FootXY_trajectory(t_temp, interval-dt_main,pos_start_FR.x(), pos_end_FR.x(), 0, x_dx_ddx_old[FR]);
        y_dy_ddy[FR] = FootXY_trajectory(t_temp, interval-dt_main,pos_start_FR.y(), pos_end_FR.y(), 0, y_dy_ddy_old[FR]);
        z_dz_ddz[FR] = FootZ_Swing_trajectory(t_temp, interval-dt_main, pos_start_FR.z(), pos_end_FR.z(), step_z, z_dz_ddz_old[FR]);
        state_motion.foot.FR.pos = Vector3d(x_dx_ddx[FR].x(), y_dy_ddy[FR].x(), z_dz_ddz[FR].x());
        state_motion.foot.FR.vel = Vector3d(x_dx_ddx[FR].y(), y_dy_ddy[FR].y(), z_dz_ddz[FR].y());

        if(t_temp >= interval - dt_main) {
            //cout << "check!" << endl;
            if(contact_flag[HL] == false) {
                contact_flag[HL] = true;
            }
            if(contact_flag[FR] == false) {
                contact_flag[FR] = true;
            }

//            if(contact_flag_test[HL] == false) {
//                contact_flag_test[HL] = true;
//            }
//            if(contact_flag_test[FR] == false) {
//                contact_flag_test[FR] = true;
//            }

            Endpos_SW[HL] = state_motion.foot.HL.pos;
            Endpos_SW[FR] = state_motion.foot.FR.pos;

            Endpos_ST[HR] = state_motion.foot.HR.pos;
            Endpos_ST[FL] = state_motion.foot.FL.pos;

            contact_flag[HR] = false;
            contact_flag[FL] = false;
            contact_flag_test[HR] = false;
            contact_flag_test[FL] = false;
        }

        for(int i=0; i<4; i++){
            x_dx_ddx_old[i] = x_dx_ddx[i];
            y_dy_ddy_old[i] = y_dy_ddy[i];
            z_dz_ddz_old[i] = z_dz_ddz[i];
        }
    }

    else if(step_num == 1){} // pause

    else if(step_num == 2){ // ------------------ motion 02 ---------------------
        t_temp = t_sim-timeline[2];

        // ----------------------- Check contact -----------------------

        if(t_temp >=0.05*interval && t_temp < 0.4*interval) {
            if(PosUpdateflag == false) {
                //ComState.vel = Vector3d(0,0,0);
            }
            PosUpdateflag = true;

            zmp_offset.x() = dist_modi_step.x() + joy_acc_modi_step.x();
            zmp_offset.y() = dist_modi_step.y() + joy_acc_modi_step.y();
        }
        else PosUpdateflag = false;

        // Late Landing Check
        if(t_temp <= 1.0 - contact_checktime*interval) {
            if(QuadState.foot.HL.force.z() < contact_threshold) {
                contact_flag_test[HL] = true;

            }
            if(QuadState.foot.FR.force.z() < contact_threshold) {
                contact_flag_test[FR] = true;
            }
        }
        else {
            contact_flag_test[HL] = true;
            contact_flag_test[FR] = true;
        }

        // Early Landing Check
        if(t_temp >= contact_checktime*interval) {
            if(contact_mode_flag == true) {
                //                if(t_temp >= 0.8*interval) {
                //                    contact_flag[HR] = true;
                //                }
                //                if(t_temp >= 0.9*interval) {
                //                    contact_flag[FL] = true;
                //                }
                if(QuadState.foot.HR.force.z() < contact_threshold) {
                    contact_flag_test[HR] = true;
                }
                if(QuadState.foot.FL.force.z() < contact_threshold) {
                    contact_flag_test[FL] = true;
                }
            }
            landing_flag[HR] = true;
            landing_flag[FL] = true;
        }
        // HB version

        //HR-swing
        Vector3d pos_start_HR =  Endpos_ST[HR];
        Vector3d pos_end_HR =  QuadState_start.foot.HR.pos + contact_el_offset + 0.5*pos_step + 0.5*FootStepRotation(pos_rotate,HR) + zmp_offset;
        x_dx_ddx[HR] = FootXY_trajectory(t_temp, interval-dt_main, pos_start_HR.x(), pos_end_HR.x(), vel_joy.x()*1.5, x_dx_ddx_old[HR]);
        y_dy_ddy[HR] = FootXY_trajectory(t_temp, interval-dt_main, pos_start_HR.y(), pos_end_HR.y(), vel_joy.y(), y_dy_ddy_old[HR]);
        z_dz_ddz[HR] = FootZ_Swing_trajectory(t_temp, interval-dt_main,pos_start_HR.z(), pos_end_HR.z()+0.005, step_z, z_dz_ddz_old[HR]);
        state_motion.foot.HR.pos = Vector3d(x_dx_ddx[HR].x(), y_dy_ddy[HR].x(), z_dz_ddz[HR].x());
        state_motion.foot.HR.vel = Vector3d(x_dx_ddx[HR].y(), y_dy_ddy[HR].y(), z_dz_ddz[HR].y());

        //FL-swing
        Vector3d pos_start_FL =  Endpos_ST[FL];
        Vector3d pos_end_FL =  QuadState_start.foot.FL.pos + contact_el_offset + 0.5*pos_step + 0.5*FootStepRotation(pos_rotate,FL) + zmp_offset;
        x_dx_ddx[FL] = FootXY_trajectory(t_temp, interval-dt_main, pos_start_FL.x(), pos_end_FL.x(), vel_joy.x()*1.5, x_dx_ddx_old[FL]);
        y_dy_ddy[FL] = FootXY_trajectory(t_temp, interval-dt_main, pos_start_FL.y(), pos_end_FL.y(), vel_joy.y(), y_dy_ddy_old[FL]);
        z_dz_ddz[FL] = FootZ_Swing_trajectory(t_temp, interval-dt_main, pos_start_FL.z(), pos_end_FL.z()+0.005, step_z, z_dz_ddz_old[FL]);
        state_motion.foot.FL.pos = Vector3d(x_dx_ddx[FL].x(), y_dy_ddy[FL].x(), z_dz_ddz[FL].x());
        state_motion.foot.FL.vel = Vector3d(x_dx_ddx[FL].y(), y_dy_ddy[FL].y(), z_dz_ddz[FL].y());

        //HL-stance
        Vector3d pos_start_HL =  Endpos_SW[HL];
        Vector3d pos_end_HL =  QuadState_start.foot.HL.pos - 0.5*pos_step + 0.5*FootStepRotation(-pos_rotate,HL) + zmp_offset_pre;
        x_dx_ddx[HL] = FootXY_trajectory(t_temp, interval-dt_main, pos_start_HL.x(), pos_end_HL.x(), vel_joy.x()*1.5, x_dx_ddx_old[HL]);
        y_dy_ddy[HL] = FootXY_trajectory(t_temp, interval-dt_main, pos_start_HL.y(), pos_end_HL.y(), vel_joy.y(), y_dy_ddy_old[HL]);
        z_dz_ddz[HL] = FootXY_trajectory(t_temp, interval-dt_main, pos_start_HL.z(), pos_end_HL.z(), 0, z_dz_ddz_old[HL]);
        state_motion.foot.HL.pos = Vector3d(x_dx_ddx[HL].x(), y_dy_ddy[HL].x(), z_dz_ddz[HL].x());
        state_motion.foot.HL.vel = Vector3d(x_dx_ddx[HL].y(), y_dy_ddy[HL].y(), z_dz_ddz[HL].y());

        //FR-stance
        Vector3d pos_start_FR =  Endpos_SW[FR];
        Vector3d pos_end_FR =  QuadState_start.foot.FR.pos - 0.5*pos_step + 0.5*FootStepRotation(-pos_rotate,FR) + zmp_offset_pre;
        x_dx_ddx[FR] = FootXY_trajectory(t_temp, interval-dt_main, pos_start_FR.x(), pos_end_FR.x(), vel_joy.x()*1.5, x_dx_ddx_old[FR]);
        y_dy_ddy[FR] = FootXY_trajectory(t_temp, interval-dt_main, pos_start_FR.y(), pos_end_FR.y(), vel_joy.y(), y_dy_ddy_old[FR]);
        z_dz_ddz[FR] = FootXY_trajectory(t_temp, interval-dt_main, pos_start_FR.z(), pos_end_FR.z(), 0, z_dz_ddz_old[FR]);
        state_motion.foot.FR.pos = Vector3d(x_dx_ddx[FR].x(), y_dy_ddy[FR].x(), z_dz_ddz[FR].x());
        state_motion.foot.FR.vel = Vector3d(x_dx_ddx[FR].y(), y_dy_ddy[FR].y(), z_dz_ddz[FR].y());

        // save landing height z (contact flag changed)
        if(contact_mode_flag == true) {
            if(contact_flag_test[HR] == true && contact_flag_test_pre[HR] == false) {
                landing_height[HR] = QuadState.foot.HR.pos.z();
            }
            if(contact_flag_test[FL] == true && contact_flag_test_pre[FL] == false) {
                landing_height[FL] = QuadState.foot.FL.pos.z();
            }
            if(contact_flag_test[HL] == true && contact_flag_test_pre[HL] == false) {
                landing_height[HL] = QuadState.foot.HL.pos.z();
            }
            if(contact_flag_test[FR] == true && contact_flag_test_pre[FR] == false) {
                landing_height[FR] = QuadState.foot.FR.pos.z();
            }
        }

        if(t_temp >= interval - dt_main) {
            if(contact_flag[HR] == false) {
                contact_flag[HR] = true;
                //Endpos_SW[HR] = state_motion.foot.posHR.pos;pos_end_HL.
                //cout << "check HR 2" << endl;
            }
            if(contact_flag[FL] == false) {
                contact_flag[FL] = true;
                //Endpos_SW[FL] = state_motion.foot.FL.pos;
                //cout << "check FL 2" << endl;
            }

//            if(contact_flag_test[HR] == false) {
//                contact_flag_test[HR] = true;
//            }
//            if(contact_flag_test[FL] == false) {
//                contact_flag_test[FL] = true;
//            }

            Endpos_SW[HR] = state_motion.foot.HR.pos;
            Endpos_SW[FL] = state_motion.foot.FL.pos;

            Endpos_ST[HL] = state_motion.foot.HL.pos;
            Endpos_ST[FR] = state_motion.foot.FR.pos;

            contact_flag[HL] = false;
            contact_flag[FR] = false;
            contact_flag_test[HL] = false;
            contact_flag_test[FR] = false;

            zmp_offset_pre = zmp_offset;
        }

        for(int i=0; i<4; i++){
            x_dx_ddx_old[i] = x_dx_ddx[i];
            y_dy_ddy_old[i] = y_dy_ddy[i];
            z_dz_ddz_old[i] = z_dz_ddz[i];
        }

    }

    else if(step_num == 3){

    } //------------------------------------PAUSE--------------------------------------------------

    else if(step_num == 4){ // ------------------ motion 03 ---------------------
        t_temp = t_sim-timeline[4];

        if(t_temp >=0.05*interval && t_temp < 0.4*interval) {
            if(PosUpdateflag == false) {
            }
            PosUpdateflag = true;
            zmp_offset.x() = dist_modi_step.x() + joy_acc_modi_step.x();
            zmp_offset.y() = dist_modi_step.y() + joy_acc_modi_step.y();

        }
        else PosUpdateflag = false;

        if(t_temp <= 1.0 - contact_checktime*interval) {
            if(QuadState.foot.HR.force.z() < contact_threshold) {
                contact_flag_test[HR] = true;
            }
            if(QuadState.foot.FL.force.z() < contact_threshold) {
                contact_flag_test[FL] = true;
            }
        }
        else {
            contact_flag_test[HR] = true;
            contact_flag_test[FL] = true;
        }

        if(t_temp >= contact_checktime*interval) {
            if(contact_mode_flag == true) {
                //                if(t_temp >= 0.8*interval) {
                //                    contact_flag[HL] = true;
                //                }
                //                if(t_temp >= 0.9*interval) {
                //                    contact_flag[FR] = true;
                //                }
                if(QuadState.foot.HL.force.z() < contact_threshold) {
                    contact_flag_test[HL] = true;
                }
                if(QuadState.foot.FR.force.z() < contact_threshold) {
                    contact_flag_test[FR] = true;
                }
            }
            landing_flag[HL] = true;
            landing_flag[FR] = true;
        }

        // HB version
        //HR-stance
        Vector3d pos_start_HR =  Endpos_SW[HR];
        Vector3d pos_end_HR =  QuadState_start.foot.HR.pos - 0.5*(pos_step) + 0.5*FootStepRotation(-pos_rotate,HR) + zmp_offset_pre;
        x_dx_ddx[HR] = FootXY_trajectory(t_temp, interval-dt_main, pos_start_HR.x(), pos_end_HR.x(), vel_joy.x()*1.5, x_dx_ddx_old[HR]);
        y_dy_ddy[HR] = FootXY_trajectory(t_temp, interval-dt_main, pos_start_HR.y(), pos_end_HR.y(), vel_joy.y(), y_dy_ddy_old[HR]);
        z_dz_ddz[HR] = FootXY_trajectory(t_temp, interval-dt_main, pos_start_HR.z(), pos_end_HR.z(), 0, z_dz_ddz_old[HR]);
        state_motion.foot.HR.pos = Vector3d(x_dx_ddx[HR].x(), y_dy_ddy[HR].x(), z_dz_ddz[HR].x());
        state_motion.foot.HR.vel = Vector3d(x_dx_ddx[HR].y(), y_dy_ddy[HR].y(), z_dz_ddz[HR].y());

        //FL-stance
        Vector3d pos_start_FL =  Endpos_SW[FL];
        Vector3d pos_end_FL =  QuadState_start.foot.FL.pos - 0.5*(pos_step) + 0.5*FootStepRotation(-pos_rotate,FL) + zmp_offset_pre;
        x_dx_ddx[FL] = FootXY_trajectory(t_temp, interval-dt_main, pos_start_FL.x(), pos_end_FL.x(), vel_joy.x()*1.5, x_dx_ddx_old[FL]);
        y_dy_ddy[FL] = FootXY_trajectory(t_temp, interval-dt_main, pos_start_FL.y(), pos_end_FL.y(), vel_joy.y(), y_dy_ddy_old[FL]);
        z_dz_ddz[FL] = FootXY_trajectory(t_temp, interval-dt_main, pos_start_FL.z(), pos_end_FL.z(), 0, z_dz_ddz_old[FL]);
        state_motion.foot.FL.pos = Vector3d(x_dx_ddx[FL].x(), y_dy_ddy[FL].x(), z_dz_ddz[FL].x());
        state_motion.foot.FL.vel = Vector3d(x_dx_ddx[FL].y(), y_dy_ddy[FL].y(), z_dz_ddz[FL].y());

        //HL-swing
        Vector3d pos_start_HL =  Endpos_ST[HL];
        Vector3d pos_end_HL =  QuadState_start.foot.HL.pos + contact_el_offset + 0.5*pos_step + 0.5*FootStepRotation(pos_rotate,HL) + zmp_offset;
        x_dx_ddx[HL] = FootXY_trajectory(t_temp, interval-dt_main, pos_start_HL.x(), pos_end_HL.x(), vel_joy.x()*1.5, x_dx_ddx_old[HL]);
        y_dy_ddy[HL] = FootXY_trajectory(t_temp, interval-dt_main, pos_start_HL.y(), pos_end_HL.y(), vel_joy.y(), y_dy_ddy_old[HL]);
        z_dz_ddz[HL] = FootZ_Swing_trajectory(t_temp, interval-dt_main,pos_start_HL.z(), pos_end_HL.z()+0.005, step_z, z_dz_ddz_old[HL]);
        state_motion.foot.HL.pos = Vector3d(x_dx_ddx[HL].x(), y_dy_ddy[HL].x(), z_dz_ddz[HL].x());
        state_motion.foot.HL.vel = Vector3d(x_dx_ddx[HL].y(), y_dy_ddy[HL].y(), z_dz_ddz[HL].y());

        //FR-swing
        Vector3d pos_start_FR =  Endpos_ST[FR];
        Vector3d pos_end_FR =  QuadState_start.foot.FR.pos + contact_el_offset + 0.5*pos_step + 0.5*FootStepRotation(pos_rotate,FR) + zmp_offset;
        x_dx_ddx[FR] = FootXY_trajectory(t_temp, interval-dt_main, pos_start_FR.x(), pos_end_FR.x(), vel_joy.x()*1.5, x_dx_ddx_old[FR]);
        y_dy_ddy[FR] = FootXY_trajectory(t_temp, interval-dt_main, pos_start_FR.y(), pos_end_FR.y(), vel_joy.y(), y_dy_ddy_old[FR]);
        z_dz_ddz[FR] = FootZ_Swing_trajectory(t_temp, interval-dt_main, pos_start_FR.z(), pos_end_FR.z()+0.005, step_z, z_dz_ddz_old[FR]);
        state_motion.foot.FR.pos = Vector3d(x_dx_ddx[FR].x(), y_dy_ddy[FR].x(), z_dz_ddz[FR].x());
        state_motion.foot.FR.vel = Vector3d(x_dx_ddx[FR].y(), y_dy_ddy[FR].y(), z_dz_ddz[FR].y());


        if(contact_mode_flag == true) {
            if(contact_flag_test[HR] == true && contact_flag_test_pre[HR] == false) {
                landing_height[HR] = QuadState.foot.HR.pos.z();
            }
            if(contact_flag_test[FL] == true && contact_flag_test_pre[FL] == false) {
                landing_height[FL] = QuadState.foot.FL.pos.z();
            }
            if(contact_flag_test[HL] == true && contact_flag_test_pre[HL] == false) {
                landing_height[HL] = QuadState.foot.HL.pos.z();
            }
            if(contact_flag_test[FR] == true && contact_flag_test_pre[FR] == false) {
                landing_height[FR] = QuadState.foot.FR.pos.z();
            }
        }

        if(t_temp >= interval - dt_main) {
            //cout << "check!" << endl;
            if(contact_flag[HL] == false) {
                contact_flag[HL] = true;
                //Endpos_SW[HL] = state_motion.foot.HL.pos;
                //cout << "check HL 2" << endl;
            }
            if(contact_flag[FR] == false) {
                contact_flag[FR] = true;
                //Endpos_SW[FR] = state_motion.foot.FR.pos;
                //cout << "check FR 2" << endl;
            }

//            if(contact_flag_test[HL] == false) {
//                contact_flag_test[HL] = true;
//            }
//            if(contact_flag_test[FR] == false) {
//                contact_flag_test[FR] = true;
//            }

            Endpos_SW[HL] = state_motion.foot.HL.pos;
            Endpos_SW[FR] = state_motion.foot.FR.pos;

            Endpos_ST[HR] = state_motion.foot.HR.pos;
            Endpos_ST[FL] = state_motion.foot.FL.pos;

            contact_flag[HR] = false;
            contact_flag[FL] = false;
            contact_flag_test[HR] = false;
            contact_flag_test[FL] = false;

            zmp_offset_pre = zmp_offset;

            if(start_flag == true) {
                FirstStep = false;
                step_init_flag = true;
                cnt_motion = timeline[1]/dt_main-1;
            }
        }

        for(int i=0; i<4; i++){
            x_dx_ddx_old[i] = x_dx_ddx[i];
            y_dy_ddy_old[i] = y_dy_ddy[i];
            z_dz_ddz_old[i] = z_dz_ddz[i];
        }
    }

    else if(step_num == 5){} // pause

    else if(step_num == 6){ // ------------------ motion 04 --------------------- end motion
        t_temp = t_sim-timeline[6];

        // ----------------------- Check contact -----------------------

        if(t_temp >=0.05*interval && t_temp < 0.4*interval) {
            if(PosUpdateflag == false) {
                //ComState.vel = Vector3d(0,0,0);
            }
            PosUpdateflag = true;

            zmp_offset.x() = dist_modi_step.x() + joy_acc_modi_step.x();
            zmp_offset.y() = dist_modi_step.y() + joy_acc_modi_step.y();
        }
        else PosUpdateflag = false;

        if(t_temp <= 1.0 - contact_checktime*interval) {
            if(QuadState.foot.HL.force.z() < contact_threshold) {
                contact_flag_test[HL] = true;

            }
            if(QuadState.foot.FR.force.z() < contact_threshold) {
                contact_flag_test[FR] = true;
            }
        }
        else {
            contact_flag_test[HL] = true;
            contact_flag_test[FR] = true;
        }

        if(t_temp >= contact_checktime*interval) {
            if(contact_mode_flag == true) {
                //                if(t_temp >= 0.8*interval) {
                //                    contact_flag[HR] = true;
                //                }
                //                if(t_temp >= 0.9*interval) {
                //                    contact_flag[FL] = true;
                //                }
                if(QuadState.foot.HR.force.z() < contact_threshold) {
                    contact_flag_test[HR] = true;
                }
                if(QuadState.foot.FL.force.z() < contact_threshold) {
                    contact_flag_test[FL] = true;
                }
            }
            landing_flag[HR] = true;
            landing_flag[FL] = true;
        }
        // HB version

        //HR-swing
        Vector3d pos_start_HR =  Endpos_ST[HR];
        Vector3d pos_end_HR =  QuadState_start.foot.HR.pos;
        x_dx_ddx[HR] = FootXY_trajectory(t_temp, interval-dt_main, pos_start_HR.x(), pos_end_HR.x(), 0, x_dx_ddx_old[HR]);
        y_dy_ddy[HR] = FootXY_trajectory(t_temp, interval-dt_main, pos_start_HR.y(), pos_end_HR.y(), 0, y_dy_ddy_old[HR]);
        z_dz_ddz[HR] = FootZ_Swing_trajectory(t_temp, interval-dt_main,pos_start_HR.z(), pos_end_HR.z(), step_z, z_dz_ddz_old[HR]);
        state_motion.foot.HR.pos = Vector3d(x_dx_ddx[HR].x(), y_dy_ddy[HR].x(), z_dz_ddz[HR].x());
        state_motion.foot.HR.vel = Vector3d(x_dx_ddx[HR].y(), y_dy_ddy[HR].y(), z_dz_ddz[HR].y());

        //FL-swing
        Vector3d pos_start_FL =  Endpos_ST[FL];
        Vector3d pos_end_FL =  QuadState_start.foot.FL.pos;
        x_dx_ddx[FL] = FootXY_trajectory(t_temp, interval-dt_main, pos_start_FL.x(), pos_end_FL.x(), 0, x_dx_ddx_old[FL]);
        y_dy_ddy[FL] = FootXY_trajectory(t_temp, interval-dt_main, pos_start_FL.y(), pos_end_FL.y(), 0, y_dy_ddy_old[FL]);
        z_dz_ddz[FL] = FootZ_Swing_trajectory(t_temp, interval-dt_main, pos_start_FL.z(), pos_end_FL.z(), step_z, z_dz_ddz_old[FL]);
        state_motion.foot.FL.pos = Vector3d(x_dx_ddx[FL].x(), y_dy_ddy[FL].x(), z_dz_ddz[FL].x());
        state_motion.foot.FL.vel = Vector3d(x_dx_ddx[FL].y(), y_dy_ddy[FL].y(), z_dz_ddz[FL].y());


        //HL-stance
        Vector3d pos_start_HL =  Endpos_SW[HL];
        Vector3d pos_end_HL =  QuadState_start.foot.HL.pos;
        x_dx_ddx[HL] = FootXY_trajectory(t_temp, interval-dt_main, pos_start_HL.x(), pos_end_HL.x(), 0, x_dx_ddx_old[HL]);
        y_dy_ddy[HL] = FootXY_trajectory(t_temp, interval-dt_main, pos_start_HL.y(), pos_end_HL.y(), 0, y_dy_ddy_old[HL]);
        z_dz_ddz[HL] = FootXY_trajectory(t_temp, interval-dt_main, pos_start_HL.z(), pos_end_HL.z(), 0, z_dz_ddz_old[HL]);
        state_motion.foot.HL.pos = Vector3d(x_dx_ddx[HL].x(), y_dy_ddy[HL].x(), z_dz_ddz[HL].x());
        state_motion.foot.HL.vel = Vector3d(x_dx_ddx[HL].y(), y_dy_ddy[HL].y(), z_dz_ddz[HL].y());

        //FR-stance
        Vector3d pos_start_FR =  Endpos_SW[FR];
        Vector3d pos_end_FR =  QuadState_start.foot.FR.pos;
        x_dx_ddx[FR] = FootXY_trajectory(t_temp, interval-dt_main, pos_start_FR.x(), pos_end_FR.x(), 0, x_dx_ddx_old[FR]);
        y_dy_ddy[FR] = FootXY_trajectory(t_temp, interval-dt_main, pos_start_FR.y(), pos_end_FR.y(), 0, y_dy_ddy_old[FR]);
        z_dz_ddz[FR] = FootXY_trajectory(t_temp, interval-dt_main, pos_start_FR.z(), pos_end_FR.z(), 0, z_dz_ddz_old[FR]);
        state_motion.foot.FR.pos = Vector3d(x_dx_ddx[FR].x(), y_dy_ddy[FR].x(), z_dz_ddz[FR].x());
        state_motion.foot.FR.vel = Vector3d(x_dx_ddx[FR].y(), y_dy_ddy[FR].y(), z_dz_ddz[FR].y());

        if(t_temp >= interval - dt_main) {

            Endpos_SW[HR] = state_motion.foot.HR.pos;
            Endpos_SW[FL] = state_motion.foot.FL.pos;

            Endpos_ST[HL] = state_motion.foot.HL.pos;
            Endpos_ST[FR] = state_motion.foot.FR.pos;

            for(int i=0; i<4; i++) {
                contact_flag[i] = true;
                contact_flag_test[i] = true;
            }
            zmp_offset_pre = zmp_offset;
        }

        for(int i=0; i<4; i++){
            x_dx_ddx_old[i] = x_dx_ddx[i];
            y_dy_ddy_old[i] = y_dy_ddy[i];
            z_dz_ddz_old[i] = z_dz_ddz[i];
        }
    }

    else if(step_num == 7){} // pause

    else{
        t_temp = t_sim-timeline[8];

        if(start_flag == true) {
        }
        else {
            step_num = -1;
            FirstStep = true;
            ready_flag = true;
            start_flag = false;
            stop_flag = false;
            end_flag = false;
            onestep_flag = false;
            mode_state = 0;
            trot_init_flag = true;
            step_init_flag = true;
            cnt_motion = -1;
        }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////

    //--------------------------------------------------------------------------------------------------
    // COM control parameter

    Vector3d com_Kp_f = Vector3d(sharedData->control_variable_double[1] ,sharedData->control_variable_double[3], sharedData->control_variable_double[5]);
    Vector3d com_Kd_f = Vector3d(sharedData->control_variable_double[2] ,sharedData->control_variable_double[4], sharedData->control_variable_double[6]);

    if(trot_mode == F_TROT_WALKING) {
        com_Kp_f = Vector3d(1000.0, 1000.0, 0);
    }

    Matrix3d com_Kp_mat_f = diagonalize(com_Kp_f);
    Matrix3d com_Kd_mat_f = diagonalize(com_Kd_f);

    double angle_limit2 = 30*D2Rf;
    Vector3d IMU_angle_err_limited;
    //Vector3d ACC_angle_rad = Vector3d(sharedData->IMU[1].AccX*D2Rf, sharedData->IMU[1].AccY*D2Rf, 0);

//    Vector3d IMU_angle_error = RampEulerRef - QuadSensor.IMU.angle;

    Vector3d IMU_angle_error = Joy_Orientation + RampEulerRef - QuadSensor.IMU.angle;

//    cout << "ref : " << Joy_Orientation.y() << ", now : " << QuadSensor.IMU.angle.y() << endl;

    for(int i=0; i<3; i++) {
        if(IMU_angle_error[i] > angle_limit2) IMU_angle_err_limited[i] = angle_limit2;
        else if(IMU_angle_error[i] < -angle_limit2) IMU_angle_err_limited[i] = -angle_limit2;
        else IMU_angle_err_limited[i] = IMU_angle_error[i];

        //        if(ACC_angle_rad[i] > angle_limit2) IMU_angle_limited[i] = angle_limit2;
        //        else if(ACC_angle_rad[i] < -angle_limit2) IMU_angle_limited[i] = -angle_limit2;
        //        else IMU_angle_limited[i] = ACC_angle_rad[i];
    }

    com_control_force = com_Kp_mat_f*(IMU_angle_err_limited) + com_Kd_mat_f*-QuadSensor.IMU.vel;

    double limit_f[3] = {280, 280, 0};
    for(int i=0; i<3; i++) {
        if(com_control_force[i] > limit_f[i]) com_control_force[i] = limit_f[i];
        else if(com_control_force[i] < -limit_f[i]) com_control_force[i] = -limit_f[i];
    }

    foot_Force_COM[HR] = Vector3d(0, 0, +com_control_force.x() -com_control_force.y());
    foot_Force_COM[HL] = Vector3d(0, 0, -com_control_force.x() -com_control_force.y());
    foot_Force_COM[FR] = Vector3d(0, 0, +com_control_force.x() +com_control_force.y());
    foot_Force_COM[FL] = Vector3d(0, 0, -com_control_force.x() +com_control_force.y());

    // -------------------------tilt estimation --------------------------
    double front_height = (landing_height[FR] + landing_height[FL])/2.0;
    double hind_height = (landing_height[HR] + landing_height[HL])/2.0;

    double right_height = (landing_height[FR] + landing_height[HR])/2.0;
    double left_height = (landing_height[HL] + landing_height[FL])/2.0;

    double body_length_x = Kinematics.P2HX*2;
    double body_length_y = (Kinematics.P2HY + Kinematics.R2P)*2;

    Vector3d tilt_raw = Vector3d(atan((left_height - right_height)/body_length_y), atan((hind_height - front_height)/body_length_x), 0);

    tilt_est = Lowpass_filter_3d(tilt_raw, tilt_est, 0.5, dt_main);

    // ------------------------- Add compensation ------------------------
    if(step_num%2 == 0) {
        rotate_comp = ff_rcomp*sin(t_temp*PI/interval);
        if(step_num == 2) {
            state_comp.foot.HR.pos = FootStepRotation(rotate_comp,HR);
            state_comp.foot.HL.pos = FootStepRotation(rotate_comp,HL);
            state_comp.foot.FR.pos = FootStepRotation(rotate_comp,FR);
            state_comp.foot.FL.pos = FootStepRotation(rotate_comp,FL);
        }
        else if(step_num == 4) {
            state_comp.foot.HR.pos = -FootStepRotation(rotate_comp,HR);
            state_comp.foot.HL.pos = -FootStepRotation(rotate_comp,HL);
            state_comp.foot.FR.pos = -FootStepRotation(rotate_comp,FR);
            state_comp.foot.FL.pos = -FootStepRotation(rotate_comp,FL);
        }
    }
    // ------------------------- Position reference output ------------------------
    state_out = state_motion;

    Matrix3d tilt_rot = Euler2R(RampEulerRef);
    Vector3d state_out_foot_xy;

    state_out.rotate = tilt_rot;

    state_out_foot_xy = tilt_rot*Vector3d(state_out.foot.HR.pos.x(), state_out.foot.HR.pos.y(), 0);
    state_out.foot.HR.pos = state_out_foot_xy + Vector3d(0,0,state_motion.foot.HR.pos.z());
    state_out_foot_xy = tilt_rot*Vector3d(state_out.foot.HR.vel.x(), state_out.foot.HR.vel.y(), 0);
    state_out.foot.HR.vel = state_out_foot_xy + Vector3d(0,0,state_motion.foot.HR.vel.z());

    state_out_foot_xy = tilt_rot*Vector3d(state_out.foot.HL.pos.x(), state_out.foot.HL.pos.y(), 0);
    state_out.foot.HL.pos = state_out_foot_xy + Vector3d(0,0,state_motion.foot.HL.pos.z());
    state_out_foot_xy = tilt_rot*Vector3d(state_out.foot.HL.vel.x(), state_out.foot.HL.vel.y(), 0);
    state_out.foot.HL.vel = state_out_foot_xy + Vector3d(0,0,state_motion.foot.HL.vel.z());

    state_out_foot_xy = tilt_rot*Vector3d(state_out.foot.FR.pos.x(), state_out.foot.FR.pos.y(), 0);
    state_out.foot.FR.pos = state_out_foot_xy + Vector3d(0,0,state_motion.foot.FR.pos.z());
    state_out_foot_xy = tilt_rot*Vector3d(state_out.foot.FR.vel.x(), state_out.foot.FR.vel.y(), 0);
    state_out.foot.FR.vel = state_out_foot_xy + Vector3d(0,0,state_motion.foot.FR.vel.z());

    state_out_foot_xy = tilt_rot*Vector3d(state_out.foot.FL.pos.x(), state_out.foot.FL.pos.y(), 0);
    state_out.foot.FL.pos = state_out_foot_xy + Vector3d(0,0,state_motion.foot.FL.pos.z());
    state_out_foot_xy = tilt_rot*Vector3d(state_out.foot.FL.vel.x(), state_out.foot.FL.vel.y(), 0);
    state_out.foot.FL.vel = state_out_foot_xy + Vector3d(0,0,state_motion.foot.FL.vel.z());

    state_out.foot.HR.pos += state_comp.foot.HR.pos;
    state_out.foot.HL.pos += state_comp.foot.HL.pos;
    state_out.foot.FR.pos += state_comp.foot.FR.pos;
    state_out.foot.FL.pos += state_comp.foot.FL.pos;

    // ------------------------- COM Move ------------------------
    //        double com_mov_x = Joy_X/0.3;
    //        double com_mov_y = Joy_Y/0.15;


    CoM_height_variation = Lowpass_filter(fabs(vel_joy.x())*interval*interval*interval*1.3, CoM_height_variation,1.0,dt_main);

    state_out.COM.pos.z() = sharedData->z0 - CoM_height_variation;
    state_out.COM.pos.x() = sharedData->x0;
    state_out.COM.pos.y() = 0; //Joy_ComZ*0.3;

    state_out.rotate = Euler2R(Joy_Orientation);



    //        state_out.foot.HR.pos.y() += -speed_comp*0.04;
    //        state_out.foot.HL.pos.y() += +speed_comp*0.04;
    //        state_out.foot.FR.pos.y() += +speed_comp*0.04;
    //        state_out.foot.FL.pos.y() += -speed_comp*0.04;

    //Joy_ComZ = Lowpass_filter(sharedData->joy_dpCOM[2], Joy_ComZ, 0.5, dt_main);
    //state_out.COM.pos.z() = sharedData->z0 + Joy_ComZ;
    //state_out.rotate = Euler2R(com_control_force);


    // ------------------------ Task space position control --------------------
    for(int i=0; i<3; i++){
        Kp[i] = sharedData->Kp[i];
        Kd[i] = sharedData->Kd[i];
    }

    Kp_mat = diagonalize(Kp);
    Kd_mat = diagonalize(Kd);

    Matrix3d Kp_mat_task[4];
    Matrix3d Kd_mat_task[4];

    for(int i=0; i<4; i++) {
        if(start_flag == true) {

            if(landing_flag[i] == true) {
                if(trot_mode == TROT_WALKING) Kp_mat_task[i] = Matrix3d(Kp[0],0,0, 0,Kp[1],0, 0,0,100);
                else if(trot_mode == F_TROT_WALKING) Kp_mat_task[i] = Matrix3d(Kp[0],0,0, 0,Kp[1],0, 0,0,1000);
                else if(trot_mode == TROT_WALKING_S) Kp_mat_task[i] = Matrix3d(Kp[0],0,0, 0,Kp[1],0, 0,0,100);
    //            Kd_mat_task[i] = Matrix3d(Kd[0],0,0, 0,Kd[1],0, 0,0,Kd[2]/5);
                Kd_mat_task[i] = Kd_mat;
                for(int j=0; j<3; j++) {
                    MY_CONTROL_FB_GAIN[i*3+j] = 10;
                }
            }
            else if(contact_flag[i] == true) {
                if(trot_mode == TROT_WALKING) Kp_mat_task[i] = Matrix3d(Kp[0]/6*4,0,0, 0,Kp[1]/6*4,0, 0,0,400);
                else if(trot_mode == F_TROT_WALKING) Kp_mat_task[i] = Matrix3d(Kp[0],0,0, 0,Kp[1],0, 0,0,1000);
                else if(trot_mode == TROT_WALKING_S) Kp_mat_task[i] = Matrix3d(Kp[0]/6*4,0,0, 0,Kp[1]/6*4,0, 0,0,400);
                //Kp_mat_task[i] = Matrix3d(Kp[0]/6*4,0,0, 0,Kp[1]/6*4,0, 0,0,Kp[2]/6*4);
    //            Kp_mat_task[i] = Matrix3d(Kp[0],0,0, 0,Kp[1],0, 0,0,100);
                Kd_mat_task[i] = Kd_mat;
    //            Kd_mat_task[i] = Kd_mat;
                for(int j=0; j<3; j++) {
                    MY_CONTROL_FB_GAIN[i*3+j] = 10;
                }
            }
            else {
                Kp_mat_task[i] = Kp_mat;
                Kd_mat_task[i] = Kd_mat;
                for(int j=0; j<3; j++) {
                    MY_CONTROL_FB_GAIN[i*3+j] = 10;
                }
            }
        }
        else {

            Kp_mat_task[i] = Kp_mat;
            Kd_mat_task[i] = Kd_mat;
            for(int j=0; j<3; j++) {
                MY_CONTROL_FB_GAIN[i*3+j] = 10;
            }
        }
    }

    foot_Force_ref[HR] = Kp_mat_task[HR]*(state_out.foot.HR.pos-QuadState.foot.HR.pos) + Kd_mat*(state_out.foot.HR.vel - QuadState.foot.HR.vel);
    foot_Force_ref[HL] = Kp_mat_task[HL]*(state_out.foot.HL.pos-QuadState.foot.HL.pos) + Kd_mat*(state_out.foot.HL.vel - QuadState.foot.HL.vel);
    foot_Force_ref[FR] = Kp_mat_task[FR]*(state_out.foot.FR.pos-QuadState.foot.FR.pos) + Kd_mat*(state_out.foot.FR.vel - QuadState.foot.FR.vel);
    foot_Force_ref[FL] = Kp_mat_task[FL]*(state_out.foot.FL.pos-QuadState.foot.FL.pos) + Kd_mat*(state_out.foot.FL.vel - QuadState.foot.FL.vel);


    //        printf("%d, %.3f, flag : %d, %d, %d, %d\n",step_num, t_sim, landing_flag[HR], landing_flag[HL], landing_flag[FR], landing_flag[FL]);

    //        printf("%.3f, pos : %.3f\n", t_sim, QuadSensor.Encoder.pos.HRK);


    // ------------------------ Gravity compensation (contact based) --------------------

    double mg_fz = -sharedData->trot_variable_double[8];
    double fz_speed = sharedData->trot_variable_double[9];
    n_contact = contact_flag[HR]+contact_flag[HL]+contact_flag[FR]+contact_flag[FL];

    if(trot_mode == F_TROT_WALKING) {
        mg_fz = -290.0;
        fz_speed = 20.0;
    }

    for(int i=0; i<4; i++) {

        if(contact_flag[i]) {

            foot_Force_FF[i].z() -= fz_speed;
            if(n_contact_pre == 4 && n_contact == 2) {
                foot_Force_FF[i].z() = mg_fz/n_contact;
            }
            else {
                if(foot_Force_FF[i].z() < mg_fz/n_contact) foot_Force_FF[i].z() = mg_fz/n_contact;
            }
            foot_Force_COM[i].z() /= n_contact;
            if(FirstStep == false) {
//                if(i == 0){
//                    foot_Force_FF[i].x() = acc_joy.x() + state_out.foot.HR.vel.x()*x_temp;
//                    foot_Force_FF[i].y() = vel_joy.y() + state_out.foot.HR.vel.y()*y_temp;
//                }
//                if(i == 1){
//                    foot_Force_FF[i].x() = acc_joy.x() + state_out.foot.HL.vel.x()*x_temp;
//                    foot_Force_FF[i].y() = vel_joy.y() + state_out.foot.HL.vel.y()*y_temp;
//                }
//                if(i == 2){
//                    foot_Force_FF[i].x() = acc_joy.x() + state_out.foot.FR.vel.x()*x_temp;
//                    foot_Force_FF[i].y() = vel_joy.y() + state_out.foot.FR.vel.y()*y_temp;
//                }
//                if(i == 3){
//                    foot_Force_FF[i].x() = acc_joy.x() + state_out.foot.FL.vel.x()*x_temp;
//                    foot_Force_FF[i].y() = vel_joy.y() + state_out.foot.FL.vel.y()*y_temp;
//                }

                double x_FF_force_gain = sharedData->trot_variable_double[22];
                double y_FF_force_gain = sharedData->trot_variable_double[23];
                double z_FF_force_gain = sharedData->trot_variable_double[24];

                foot_Force_FF[i].x() = vel_joy.x()*x_FF_force_gain + CoM_acc_FF.x()*z_temp;
                foot_Force_FF[i].y() = vel_joy.y()*y_FF_force_gain + CoM_acc_FF.y()*z_FF_force_gain;

            }
        }
        else {
            foot_Force_COM[i].z() = 0;
            foot_Force_FF[i].x() = 0;
            foot_Force_FF[i].y() = 0;
            foot_Force_FF[i].z() = 0;
        }
    }

    n_contact_pre = n_contact;

    //        printf("step : %d,  %3.2f %3.2f %3.2f %3.2f\n", step_num, foot_Force_FF[HR].z(),foot_Force_FF[HL].z(),foot_Force_FF[FR].z(),foot_Force_FF[FL].z());

    // ------------------------ VEL, ACC FF force --------------------



//    for(int i=0; i<4; i++) {
//        foot_Force_FF[i].x() = Joy_X_acc * x_temp;
//        foot_Force_FF[i].y() = Joy_Y_acc * y_temp;
//    }

//        sharedData->comcon_variable_plot[0] = vel_joy.x();
//        sharedData->comcon_variable_plot[1] = foot_Force_FF[0].x();

//        sharedData->comcon_variable_plot[2] = vel_joy.y();
//        sharedData->comcon_variable_plot[3] = foot_Force_1FF[0].y();

//    sharedData->comcon_variable_plot[0] = ComState.vel.y();
//    sharedData->comcon_variable_plot[1] = ComStateRef.vel.y();

//    sharedData->comcon_variable_plot[2] = com_foot_step.x();
//    sharedData->comcon_variable_plot[3] = com_foot_step.y();

//    sharedData->comcon_variable_plot[4] = ComState.vel.y() + vel_joy_filtered.y();
//    sharedData->comcon_variable_plot[5] = ComState.vel.y() - ComStateRef.vel.y();

//    sharedData->comcon_variable_plot[0] = sharedData->IMU[0].Roll;
//    sharedData->comcon_variable_plot[1] = sharedData->IMU[0].AccX;

//    sharedData->comcon_variable_plot[2] = sharedData->IMU[0].Pitch;
//    sharedData->comcon_variable_plot[3] = sharedData->IMU[0].AccY;

    // ------------------------ Feedforward force contraints --------------------
    Vector3d FF_temp[4];
    for(int i=0 ; i<4 ; i++){
        FF_temp[i] = foot_Force_FF[i] + foot_Force_COM[i];
        if(FF_temp[i].z() > 30)  FF_temp[i].z() = 30;
//        FF_temp[i].x() = 0;
//        FF_temp[i].y() = 0;
    }

    //cout << "HR y : " << FF_temp[HR].y() << ", HL y : " << FF_temp[HL].y() << endl;

    // ------------------------ SUM feedforward force --------------------
    QuadStateRef.foot.HR.force = foot_Force_ref[HR] + FF_temp[HR];
    QuadStateRef.foot.HL.force = foot_Force_ref[HL] + FF_temp[HL];
    QuadStateRef.foot.FR.force = foot_Force_ref[FR] + FF_temp[FR];
    QuadStateRef.foot.FL.force = foot_Force_ref[FL] + FF_temp[FL];

    //printf("%3.2f %3.2f %3.2f\n", foot_Force_ref[FL].x(),foot_Force_ref[FL].y(),foot_Force_ref[FL].z());

    // ------------------------ Force to Torque converting --------------------

    foot_Torque_ref[HR] = legJacobian_angleRef[HR].transpose()*QuadStateRef.foot.HR.force;
    foot_Torque_ref[HL] = legJacobian_angleRef[HL].transpose()*QuadStateRef.foot.HL.force;
    foot_Torque_ref[FR] = legJacobian_angleRef[FR].transpose()*QuadStateRef.foot.FR.force;
    foot_Torque_ref[FL] = legJacobian_angleRef[FL].transpose()*QuadStateRef.foot.FL.force;

    TorqueRef.HRR = foot_Torque_ref[HR][0];
    TorqueRef.HRP = foot_Torque_ref[HR][1];
    TorqueRef.HRK = foot_Torque_ref[HR][2];
    TorqueRef.HLR = foot_Torque_ref[HL][0];
    TorqueRef.HLP = foot_Torque_ref[HL][1];
    TorqueRef.HLK = foot_Torque_ref[HL][2];
    TorqueRef.FRR = foot_Torque_ref[FR][0];
    TorqueRef.FRP = foot_Torque_ref[FR][1];
    TorqueRef.FRK = foot_Torque_ref[FR][2];
    TorqueRef.FLR = foot_Torque_ref[FL][0];
    TorqueRef.FLP = foot_Torque_ref[FL][1];
    TorqueRef.FLK = foot_Torque_ref[FL][2];

    //printf("step:%d, time:%.3f, pos x:%.4f, vel x x:%.4f\n", step_num, t_sim, state_out.foot.HR.pos.x(), x_dx_ddx[HR].y());

    // ------------------------ Task to joint converting --------------------
    QuadJoint JointRef_temp = Kinematics.IK_FullBody(state_out);

    JointRef = QuadSensorRef.Encoder.pos;
    if(sharedData->task_legcon[HR]) {
        JointRef.HRR = JointRef_temp.HRR;
        JointRef.HRP = JointRef_temp.HRP;
        JointRef.HRK = JointRef_temp.HRK;
    }
    if(sharedData->task_legcon[HL]) {
        JointRef.HLR = JointRef_temp.HLR;
        JointRef.HLP = JointRef_temp.HLP;
        JointRef.HLK = JointRef_temp.HLK;
    }
    if(sharedData->task_legcon[FR]) {
        JointRef.FRR = JointRef_temp.FRR;
        JointRef.FRP = JointRef_temp.FRP;
        JointRef.FRK = JointRef_temp.FRK;
    }
    if(sharedData->task_legcon[FL]) {
        JointRef.FLR = JointRef_temp.FLR;
        JointRef.FLP = JointRef_temp.FLP;
        JointRef.FLK = JointRef_temp.FLK;
    }

    QuadJoint CurRef_temp = Torque2Current(TorqueRef);
    double CurRef_temp_l[12];

    CurRef_temp_l[HRR] = CurRef_temp.HRR + ComputedCurRef.HRR;
    CurRef_temp_l[HRP] = CurRef_temp.HRP + ComputedCurRef.HRP;
    CurRef_temp_l[HRK] = CurRef_temp.HRK + ComputedCurRef.HRK - (CurRef_temp.HRP + ComputedCurRef.HRP)/11;
    CurRef_temp_l[HLR] = CurRef_temp.HLR + ComputedCurRef.HLR;
    CurRef_temp_l[HLP] = CurRef_temp.HLP + ComputedCurRef.HLP;
    CurRef_temp_l[HLK] = CurRef_temp.HLK + ComputedCurRef.HLK - (CurRef_temp.HLP + ComputedCurRef.HLP)/11;
    CurRef_temp_l[FRR] = CurRef_temp.FRR + ComputedCurRef.FRR;
    CurRef_temp_l[FRP] = CurRef_temp.FRP + ComputedCurRef.FRP;
    CurRef_temp_l[FRK] = CurRef_temp.FRK + ComputedCurRef.FRK - (CurRef_temp.FRP + ComputedCurRef.FRP)/11;
    CurRef_temp_l[FLR] = CurRef_temp.FLR + ComputedCurRef.FLR;
    CurRef_temp_l[FLP] = CurRef_temp.FLP + ComputedCurRef.FLP;
    CurRef_temp_l[FLK] = CurRef_temp.FLK + ComputedCurRef.FLK - (CurRef_temp.FLP + ComputedCurRef.FLP)/11;

    current_limit_ = 30;

    for(int i=0; i<12; i++) {
        if(CurRef_temp_l[i] > current_limit_) CurRef_temp_l[i] = current_limit_;
        else if(CurRef_temp_l[i] < -current_limit_) CurRef_temp_l[i] = -current_limit_;
    }

    CurRef.HRR = CurRef_temp_l[HRR];
    CurRef.HRP = CurRef_temp_l[HRP];
    CurRef.HRK = CurRef_temp_l[HRK];
    CurRef.HLR = CurRef_temp_l[HLR];
    CurRef.HLP = CurRef_temp_l[HLP];
    CurRef.HLK = CurRef_temp_l[HLK];
    CurRef.FRR = CurRef_temp_l[FRR];
    CurRef.FRP = CurRef_temp_l[FRP];
    CurRef.FRK = CurRef_temp_l[FRK];
    CurRef.FLR = CurRef_temp_l[FLR];
    CurRef.FLP = CurRef_temp_l[FLP];
    CurRef.FLK = CurRef_temp_l[FLK];


    // ---------------------------- gain scheduling -------------------------------------------
    for(int i=0; i<4; i++) {
        if(sharedData->task_legcon[i]) {
            for(int j=0; j<3; j++) {
                //MY_CONTROL_FB_GAIN[i*3+j] = 10;
                MY_CONTROL_FF_GAIN[i*3+j] = 10;
            }

        }
        else {
            for(int j=0; j<3; j++) {
                //MY_CONTROL_FB_GAIN[i*3+j] = 10;
                MY_CONTROL_FF_GAIN[i*3+j] = 0;
            }
        }
    }

    //printf("step:%d, time:%.3f, cnt:%d, vel x:%.3f, acc x:%.3f\n", step_num, t_sim, cnt_motion, ComStateRef.vel.x(), ComStateRef.acc.x());

    sharedData->contact_flag[HR] = contact_flag[HR];
    sharedData->contact_flag[HL] = contact_flag[HL];
    sharedData->contact_flag[FR] = contact_flag[FR];
    sharedData->contact_flag[FL] = contact_flag[FL];

    sharedData->contact_flag_test[HR] = contact_flag_test[HR];
    sharedData->contact_flag_test[HL] = contact_flag_test[HL];
    sharedData->contact_flag_test[FR] = contact_flag_test[FR];
    sharedData->contact_flag_test[FL] = contact_flag_test[FL];

    if(sharedData->isFalled == true) {
        task_idx = NoTask;
        sharedData->NEWCOMMAND = true;
        sharedData->COMMAND.USER_COMMAND = MOTION_FALLING;
        JointRef = QuadSensorRef.Encoder.pos;

        trot_sinit_flag = true;
        trot_init_flag = true;

        cnt_motion = -1;

        cout << "Motion Falled." << endl;

//        sharedData->plot_play_ON = false;
        isMotionEnabled = false;

        return JointRef;
    }

    if(sharedData->isStopping == true) {
        trot_sinit_flag = true;
        trot_init_flag = true;

        cnt_motion = -1;
        //isMotionStopping = false;
        //sharedData->isStopping = false;

        cout << "Motion Finished." << endl;

//        sharedData->plot_play_ON = false;
        isMotionEnabled = false;

        sharedData->incGain_flag = false;


        task_idx = NoTask;

        JointRef = QuadSensorRef.Encoder.pos;
        //JointRef = QuadSensor.Encoder.pos;
        return JointRef;
    }

    if((sharedData->COMMAND.USER_COMMAND != MOTION_STOP && ready_flag != true))
        cnt_motion++;

    //printf("step:%d, time:%.3f, cnt:%d, ready:%d, mode:%d\n", step_num, t_sim, cnt_motion, ready_flag, sharedData->trot_variable_int[0]);




//        sharedData->comcon_variable_plot[0] = QuadState.foot.HR.force.z();
//        sharedData->comcon_variable_plot[1] = QuadStateRef.foot.HR.force.z();
//        sharedData->comcon_variable_plot[2] = QuadStateRef.foot.HR.force.z() - QuadState.foot.HR.force.z();




    sharedData->comcon_variable_plot[0] = w_vel_filtered.x();
    sharedData->comcon_variable_plot[1] = w_vel_filtered.y();

    sharedData->comcon_variable_plot[2] = w_vel_body.x();
    sharedData->comcon_variable_plot[3] = w_vel_body.y();

//    sharedData->comcon_variable_plot[0] = estimator2.position.x();
//    sharedData->comcon_variable_plot[1] = estimator2.position.y();

//    sharedData->comcon_variable_plot[2] = estimator2.vWorld.x();
//    sharedData->comcon_variable_plot[3] = estimator2.vWorld.y();

//    sharedData->comcon_variable_plot[0] = ComState.vel.x();
//    sharedData->comcon_variable_plot[1] = ComState.vel.y();
//    sharedData->comcon_variable_plot[2] = ComState.pos.x();
//    sharedData->comcon_variable_plot[3] = ComState.pos.y();

    sharedData->comcon_variable_plot[4] = ComState.vel.x();
    sharedData->comcon_variable_plot[5] = ComState.vel.y();

    return  JointRef;
}
