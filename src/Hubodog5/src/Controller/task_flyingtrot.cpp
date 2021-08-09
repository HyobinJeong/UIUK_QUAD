#include "hubodog5_general.h"
#include "func_sensor.h"
#include "RBSharedMemory.h"
#include "ESTIMATOR/estimator.h"

extern RBMotorController   _DEV_MC[MAX_MC];
extern int MY_CONTROL_FB_GAIN[12];
extern int MY_CONTROL_FF_GAIN[12];

int task_start_flag = true;
int walking_start_flag = true;
int cycle_change_flag = true;
int step_change_flag = true;

Vector3d com_foot_step_hpf_FT, com_foot_step_bpf_FT;
Vector3d imu_angle_hpf_FT;

Vector3d zmp_offset_FT[4], zmp_offset_pre_FT[4];


Estimator estimator_FT(RT_TIMER_PERIOD_MS / 1000);

QuadJoint hubodog5_general::task_flyingtrot() {

    // parameters
    Vector3d ff_dxy = Vector3d(sharedData->trot_variable_double[2], sharedData->trot_variable_double[3], 0);
    double ff_dw = sharedData->trot_variable_double[4];

    // Make time line
    int num_motion = 4;
    double timeline[4][num_motion+1];

    double timeduty = 0.2;
    double time_overlap = (double)(((int)(interval/2*timeduty/dt_main))*dt_main);

    for(int i=0; i<4; i++) {timeline[i][0] = 0.0; timeline[i][num_motion] = interval*num_motion;}

    timeline[HR][1] = interval*1 - time_overlap;
    timeline[HL][1] = interval*1 + time_overlap;
    timeline[FR][1] = interval*1 + time_overlap;
    timeline[FL][1] = interval*1 - time_overlap;

    timeline[HR][2] = interval*2 + time_overlap;
    timeline[HL][2] = interval*2 - time_overlap;
    timeline[FR][2] = interval*2 - time_overlap;
    timeline[FL][2] = interval*2 + time_overlap;

    timeline[HR][3] = interval*3 - time_overlap;
    timeline[HL][3] = interval*3 + time_overlap;
    timeline[FR][3] = interval*3 + time_overlap;
    timeline[FL][3] = interval*3 - time_overlap;


    // Get walking command
    mode_state_task_pre = mode_state_task;
    mode_state_pre = mode_state;
    mode_state_task = sharedData->flyingtrot_variable_int[0]; // change variable!!!
    if(mode_state_task != mode_state_task_pre) mode_state = mode_state_task;

    ready_flag = false; start_flag = false; stop_flag = false; onestep_flag = false;
    if(mode_state == READY) ready_flag = true;
    else if(mode_state == START) start_flag = true;
    else if(mode_state == STOP) stop_flag = true;

    step_z = 0.08;

    int kp_r = sharedData->Kp_j[0];
    int kp_p = sharedData->Kp_j[1];
    int kp_k = sharedData->Kp_j[2];
    int kd_r = sharedData->Kd_j[0];
    int kd_p = sharedData->Kd_j[1];
    int kd_k = sharedData->Kd_j[2];

    /// ----------------------------------------------------- Init Varialbe ------------------------------------------------------

    // Init variable - task start
    if(task_start_flag == true) {
        task_start_flag = false;

        // Set joint gain
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

        Endpos_ST[HR] = QuadState_start.foot.HR.pos; Endpos_ST[FL] = QuadState_start.foot.FL.pos;
        Endpos_ST[HL] = QuadState_start.foot.HL.pos; Endpos_ST[FR] = QuadState_start.foot.FR.pos;

        Endpos_SW[HR] = QuadState_start.foot.HR.pos; Endpos_SW[FL] = QuadState_start.foot.FL.pos;
        Endpos_SW[HL] = QuadState_start.foot.HL.pos; Endpos_SW[FR] = QuadState_start.foot.FR.pos;

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

        imu_angle_hpf_FT = Vector3dZero;


        for(int i=0; i<4; i++){
            foot_Force_FF[i] = Vector3dZero;

            foot_Force_COM[i] = Vector3dZero;

            foot_Force_ref[i] = Vector3dZero;

            com_control_force = Vector3dZero;

            zmp_offset_FT[i] = zmp_offset_pre_FT[i] = Vector3dZero;
        }

        vel_joy = Vector3dZero;
        acc_joy = Vector3dZero;

        ComState.vel = Vector3dZero;
        ComStateRef.vel = Vector3dZero;

        ComStateRef_old.pos = Vector3dZero;
        ComState_old.pos = Vector3dZero;
        ComStateRef.pos = Vector3dZero;
        ComState.pos = Vector3dZero;
        ComLast_vel  = Vector3dZero;

        Contact_point = ComState.pos;
        Contact_point_ref = ComState.pos;

        FirstStep = true;
        ready_flag = true;
        start_flag = false;
        stop_flag = false;
        end_flag = false;
        onestep_flag = false;
        mode_state = 0;

        pos_step = Vector3dZero;



        //estimator init
        Eigen::Vector3d init_pos(0, 0, 0.28);
        Eigen::VectorXd init_joint(12);

        init_joint << QuadSensor.Encoder.pos.HRR, QuadSensor.Encoder.pos.HRP, QuadSensor.Encoder.pos.HRK,
                      QuadSensor.Encoder.pos.HLR, QuadSensor.Encoder.pos.HLP, QuadSensor.Encoder.pos.HLK,
                      QuadSensor.Encoder.pos.FRR, QuadSensor.Encoder.pos.FRP, QuadSensor.Encoder.pos.FRK,
                      QuadSensor.Encoder.pos.FLR, QuadSensor.Encoder.pos.FLP, QuadSensor.Encoder.pos.FLK;

        estimator_FT.Init(init_pos, init_joint, QuadSensor.IMU.quat);


        cout << "trot start init" << endl;


    }
    // Init variable - walking start
    if(walking_start_flag == true) {
        walking_start_flag = false;

        for(int lnum=0; lnum<4; lnum++) contact_flag[lnum] = true;
    }
    // Init variable - every cycle
    if(cycle_change_flag == true) {
        cycle_change_flag = false;
    }
    // Init variable - every step
    if(cycle_change_flag == true) {
        cycle_change_flag = false;
    }
    // Init variable - every tic
    landing_flag[HR] = false;
    landing_flag[FL] = false;
    landing_flag[HL] = false;
    landing_flag[FR] = false;

    // init every end of cycle
    if(step_change_flag == true) {
        step_change_flag = false;
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

        interval = 0.25;

        FirstStep = false;

    }

    ///-----------------------------------------------State Estimation ----------------------------------------------------
    // Estimator
    Eigen::Matrix3d body_rot;
    Operation::quatToRotation(QuadSensor.IMU.quat, body_rot);

    Eigen::VectorXd joint_pos(12), joint_vel(12), joint_input(12);

    Eigen::Vector4i contact(4); contact<< contact_flag[0], contact_flag[1], contact_flag[2], contact_flag[3];

    joint_pos << QuadSensor.Encoder.pos.HRR, QuadSensor.Encoder.pos.HRP, QuadSensor.Encoder.pos.HRK,
                 QuadSensor.Encoder.pos.HLR, QuadSensor.Encoder.pos.HLP, QuadSensor.Encoder.pos.HLK,
                 QuadSensor.Encoder.pos.FRR, QuadSensor.Encoder.pos.FRP, QuadSensor.Encoder.pos.FRK,
                 QuadSensor.Encoder.pos.FLR, QuadSensor.Encoder.pos.FLP, QuadSensor.Encoder.pos.FLK;

    joint_vel << QuadSensor.Encoder.vel.HRR, QuadSensor.Encoder.vel.HRP, QuadSensor.Encoder.vel.HRK,
                 QuadSensor.Encoder.vel.HLR, QuadSensor.Encoder.vel.HLP, QuadSensor.Encoder.vel.HLK,
                 QuadSensor.Encoder.vel.FRR, QuadSensor.Encoder.vel.FRP, QuadSensor.Encoder.vel.FRK,
                 QuadSensor.Encoder.vel.FLR, QuadSensor.Encoder.vel.FLP, QuadSensor.Encoder.vel.FLK;

    joint_vel << QuadSensor.Encoder.current.HRR, QuadSensor.Encoder.current.HRP, QuadSensor.Encoder.current.HRK,
                 QuadSensor.Encoder.current.HLR, QuadSensor.Encoder.current.HLP, QuadSensor.Encoder.current.HLK,
                 QuadSensor.Encoder.current.FRR, QuadSensor.Encoder.current.FRP, QuadSensor.Encoder.current.FRK,
                 QuadSensor.Encoder.current.FLR, QuadSensor.Encoder.current.FLP, QuadSensor.Encoder.current.FLK;


    Eigen::Vector3d w_pos_body, w_vel_body;

    estimator_FT.estimate(body_rot,QuadSensor.IMU.acc, QuadSensor.IMU.vel, contact, joint_pos, joint_vel, w_pos_body, w_vel_body);




    /// ===================================Joy Stick-======================================

    Joy_X_pre = Joy_X;
    Joy_Y_pre = Joy_Y;
    Joy_R_pre = Joy_R;

    Joy_X = Lowpass_filter(sharedData->joy_dpCOM[0], Joy_X, 1.0, dt_main); // 5.0
    Joy_Y = Lowpass_filter(sharedData->joy_dpCOM[1], Joy_Y, 1.0, dt_main);
    Joy_R =  Lowpass_filter(sharedData->joy_dpCOM[3], Joy_R, 1.0, dt_main);

    if(start_flag == true){
        Joy_step = Vector3d(Joy_X, Joy_Y, 0);
        Joy_rotation = -Joy_R;


        Joy_Roll = 0;
        Joy_Pitch = 0;
        Joy_Yaw = 0;
        Joy_Roll_pre = 0;
        Joy_Pitch_pre = 0;
        Joy_Yaw_pre = 0;
    }

    pos_joy = Joy_step + ff_dxy;

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

    //// ====================================Step Modification===========================

    // tilt stepping
    Vector3d com_Kp_p = Vector3d(sharedData->trot_variable_double[11] ,sharedData->trot_variable_double[13], sharedData->trot_variable_double[15]);
    Vector3d com_Kd_p = Vector3d(sharedData->trot_variable_double[12] ,sharedData->trot_variable_double[14], sharedData->trot_variable_double[16]);

    Matrix3d com_Kp_mat_p = diagonalize(com_Kp_p);
    Matrix3d com_Kd_mat_p = diagonalize(com_Kd_p);

    double hpf_frq = sharedData->control_variable_double[0];

    imu_angle_hpf_FT.x() = Highpass_filter(QuadSensor.IMU.vel.x()*dt_main, 0, imu_angle_hpf_FT.x(),hpf_frq,dt_main);
    imu_angle_hpf_FT.y() = Highpass_filter(QuadSensor.IMU.vel.y()*dt_main, 0, imu_angle_hpf_FT.y(),hpf_frq,dt_main);
    imu_angle_hpf_FT.z() = Highpass_filter(QuadSensor.IMU.vel.z()*dt_main, 0, imu_angle_hpf_FT.z(),hpf_frq,dt_main);

    double angle_limit = 2.0*D2Rf;
    Vector3d imu_angle_hpf_l;
    for(int i=0; i<3; i++) {
        if(imu_angle_hpf_FT[i] > angle_limit) {
            imu_angle_hpf_l[i] = imu_angle_hpf_FT[i] - angle_limit;
        }
        else if(imu_angle_hpf_FT[i] < -angle_limit) {
            imu_angle_hpf_l[i] = imu_angle_hpf_FT[i] + angle_limit;
        }
        else imu_angle_hpf_l[i] = 0;
    }

    com_foot_step_hpf_FT = com_Kp_mat_p*-imu_angle_hpf_l;

    com_foot_step_bpf_FT.x() = Lowpass_filter(com_foot_step_hpf_FT.x(), com_foot_step_bpf_FT.x(), 0.2, dt_main);
    com_foot_step_bpf_FT.y() = Lowpass_filter(com_foot_step_hpf_FT.y(), com_foot_step_bpf_FT.y(), 0.2, dt_main);
    com_foot_step_bpf_FT.z() = Lowpass_filter(com_foot_step_hpf_FT.z(), com_foot_step_bpf_FT.z(), 0.2, dt_main);

    double limit_p[3] = {0.1, 0.1, 20*D2Rf};
    for(int i=0; i<3; i++) {
        if(com_foot_step_bpf_FT[i] > limit_p[i]) com_foot_step_bpf_FT[i] = limit_p[i];
        else if(com_foot_step_bpf_FT[i] < -limit_p[i]) com_foot_step_bpf_FT[i] = -limit_p[i];
    }

    //

    //// ===================================total foot position==========================
    pos_step = Vector3d(-com_foot_step_bpf_FT.y(),  + com_foot_step_bpf_FT.x(), 0) + pos_joy;

    pos_step.z() = 0.0;


    for(int lnum=0; lnum<4; lnum++) {
        if(cnt_motion > 0) {
            t_step_leg[lnum] += dt_main;
        }
//        else {
//            t_step_leg[lnum] = 0;
//        }

        if(t_step_leg[lnum] < 0)                                                            step_num_leg[lnum] = -1;
        else if(t_step_leg[lnum]< timeline[lnum][1])                                        step_num_leg[lnum] = 0;
        else if(t_step_leg[lnum]>=timeline[lnum][1] && t_step_leg[lnum]<timeline[lnum][2])  step_num_leg[lnum] = 1;
        else if(t_step_leg[lnum]>=timeline[lnum][2] && t_step_leg[lnum]<timeline[lnum][3])  step_num_leg[lnum] = 2;
        else if(t_step_leg[lnum]>=timeline[lnum][3] && t_step_leg[lnum]<timeline[lnum][4])  step_num_leg[lnum] = 3;
        else                                                                                step_num_leg[lnum] = 4;

        double duration[4][num_motion];
        duration[lnum][0] = timeline[lnum][1];
        duration[lnum][1] = timeline[lnum][2] - timeline[lnum][1];
        duration[lnum][2] = timeline[lnum][3] - timeline[lnum][2];
        duration[lnum][3] = timeline[lnum][4] - timeline[lnum][3];




        if(step_num_leg[lnum] == -1) {
            contact_flag[lnum] = true;
        }
        else if(step_num_leg[lnum] == 0) {
            t_temp_leg[lnum] = t_step_leg[lnum]-timeline[lnum][0];
            //trajectory 0 - starting
            contact_flag[lnum] = true;
            if(start_flag == true) {
                if(t_temp_leg[lnum] >= duration[lnum][1]-dt_main){
                    if(lnum == HL){ //swing -> stance
                        Endpos_ST[HL] = state_motion.foot.HL.pos;
                        contact_flag[HL] = true;
                    }
                    if(lnum == FR){ //swing -> stance
                        Endpos_ST[FR] = state_motion.foot.FR.pos;
                        contact_flag[FR] = true;
                    }
                    if(lnum == HR){ //stance->swing
                        Endpos_SW[HR] = state_motion.foot.HR.pos;
                        contact_flag[HR] = false;
                    }
                    if(lnum == FL){ //stance->swing
                        Endpos_SW[FL] = state_motion.foot.FL.pos;
                        contact_flag[FL] = false;
                    }
                }
            }
        }
        else if(step_num_leg[lnum] == 1) {
            t_temp_leg[lnum] = t_step_leg[lnum]-timeline[lnum][1];

            //trajectory 1
            if(lnum == HR) {
                // HR swing leg
                Vector3d pos_start =  Endpos_ST[lnum];
                Vector3d pos_end =  QuadState_start.foot.HR.pos + 0.5*pos_step + 0.5*FootStepRotation(pos_rotate,lnum);
                x_dx_ddx[lnum] = FootXY_trajectory(t_temp_leg[lnum], duration[lnum][1]-dt_main, pos_start.x(), pos_end.x(), vel_joy.x(), x_dx_ddx_old[lnum]);
                y_dy_ddy[lnum] = FootXY_trajectory(t_temp_leg[lnum], duration[lnum][1]-dt_main, pos_start.y(), pos_end.y(), vel_joy.y(), y_dy_ddy_old[lnum]);
                z_dz_ddz[lnum] = FootZ_Swing_Jump_trajectory(t_temp_leg[lnum], duration[lnum][1]-dt_main, pos_start.z(), pos_end.z(), step_z, z_dz_ddz_old[lnum]);
                state_motion.foot.HR.pos = Vector3d(x_dx_ddx[lnum].x(), y_dy_ddy[lnum].x(), z_dz_ddz[lnum].x());
                state_motion.foot.HR.vel = Vector3d(x_dx_ddx[lnum].y(), y_dy_ddy[lnum].y(), z_dz_ddz[lnum].y());

            }
            if(lnum == FL) {
                // FL swing leg
                Vector3d pos_start =  Endpos_ST[lnum];
                Vector3d pos_end =  QuadState_start.foot.FL.pos + 0.5*pos_step + 0.5*FootStepRotation(pos_rotate,lnum);
                x_dx_ddx[lnum] = FootXY_trajectory(t_temp_leg[lnum], duration[lnum][1]-dt_main, pos_start.x(), pos_end.x(), vel_joy.x(), x_dx_ddx_old[lnum]);
                y_dy_ddy[lnum] = FootXY_trajectory(t_temp_leg[lnum], duration[lnum][1]-dt_main, pos_start.y(), pos_end.y(), vel_joy.y(), y_dy_ddy_old[lnum]);
                z_dz_ddz[lnum] = FootZ_Swing_Jump_trajectory(t_temp_leg[lnum], duration[lnum][1]-dt_main, pos_start.z(), pos_end.z(), step_z, z_dz_ddz_old[lnum]);
                state_motion.foot.FL.pos = Vector3d(x_dx_ddx[lnum].x(), y_dy_ddy[lnum].x(), z_dz_ddz[lnum].x());
                state_motion.foot.FL.vel = Vector3d(x_dx_ddx[lnum].y(), y_dy_ddy[lnum].y(), z_dz_ddz[lnum].y());
            }
            if(lnum == HL){
                // HL, FR stance leg
                Vector3d pos_start =  Endpos_SW[lnum];
                Vector3d pos_end =  QuadState_start.foot.HL.pos - 0.5*pos_step + 0.5*FootStepRotation(-pos_rotate,lnum);
                x_dx_ddx[lnum] = FootXY_trajectory(t_temp_leg[lnum], duration[lnum][1]-dt_main, pos_start.x(), pos_end.x(), vel_joy.x(), x_dx_ddx_old[lnum]);
                y_dy_ddy[lnum] = FootXY_trajectory(t_temp_leg[lnum], duration[lnum][1]-dt_main, pos_start.y(), pos_end.y(), vel_joy.y(), y_dy_ddy_old[lnum]);
                z_dz_ddz[lnum] = FootZ_Stance_Jump_trajectory(t_temp_leg[lnum], duration[lnum][1]-dt_main, pos_start.z(), 1.5, z_dz_ddz_old[lnum]);
                state_motion.foot.HL.pos = Vector3d(x_dx_ddx[lnum].x(), y_dy_ddy[lnum].x(), z_dz_ddz[lnum].x());
                state_motion.foot.HL.vel = Vector3d(x_dx_ddx[lnum].y(), y_dy_ddy[lnum].y(), z_dz_ddz[lnum].y());
            }
            if(lnum == FR){
                // HL, FR stance leg
                Vector3d pos_start =  Endpos_SW[lnum];
                Vector3d pos_end =  QuadState_start.foot.FR.pos - 0.5*pos_step + 0.5*FootStepRotation(-pos_rotate,lnum);
                x_dx_ddx[lnum] = FootXY_trajectory(t_temp_leg[lnum], duration[lnum][1]-dt_main, pos_start.x(), pos_end.x(), vel_joy.x(), x_dx_ddx_old[lnum]);
                y_dy_ddy[lnum] = FootXY_trajectory(t_temp_leg[lnum], duration[lnum][1]-dt_main, pos_start.y(), pos_end.y(), vel_joy.y(), y_dy_ddy_old[lnum]);
                z_dz_ddz[lnum] = FootZ_Stance_Jump_trajectory(t_temp_leg[lnum], duration[lnum][1]-dt_main, pos_start.z(), 1.5, z_dz_ddz_old[lnum]);
                state_motion.foot.FR.pos = Vector3d(x_dx_ddx[lnum].x(), y_dy_ddy[lnum].x(), z_dz_ddz[lnum].x());
                state_motion.foot.FR.vel = Vector3d(x_dx_ddx[lnum].y(), y_dy_ddy[lnum].y(), z_dz_ddz[lnum].y());
            }

            if(t_temp_leg[lnum] >= duration[lnum][1]-dt_main){
                if(lnum == HL){ //stance -> swing
                    Endpos_ST[HL] = state_motion.foot.HL.pos;
                    contact_flag[HL] = false;
                }
                if(lnum == FR){ //stance -> swing
                    Endpos_ST[FR] = state_motion.foot.FR.pos;
                    contact_flag[FR] = false;
                }
                if(lnum == HR){ //swing -> stance
                    Endpos_SW[HR] = state_motion.foot.HR.pos;
                    contact_flag[HR] = true;
                }
                if(lnum == FL){ //swing -> stance
                    Endpos_SW[FL] = state_motion.foot.FL.pos;
                    contact_flag[FL] = true;
                }
            }
            x_dx_ddx_old[lnum] = x_dx_ddx[lnum];
            y_dy_ddy_old[lnum] = y_dy_ddy[lnum];
            z_dz_ddz_old[lnum] = z_dz_ddz[lnum];

            if(start_flag == true) {
                if(t_step_leg[lnum] >= timeline[lnum][1] - dt_main) {

                    step_change_flag = true;
                }
            }
        }

        else if(step_num_leg[lnum] == 2) {
            t_temp_leg[lnum] = t_step_leg[lnum]-timeline[lnum][2];

            //trajectory 2
            if(lnum == HL) {
                // HL swing leg
                Vector3d pos_start =  Endpos_ST[lnum];
                Vector3d pos_end =  QuadState_start.foot.HL.pos + 0.5*pos_step + 0.5*FootStepRotation(pos_rotate,lnum);
                x_dx_ddx[lnum] = FootXY_trajectory(t_temp_leg[lnum], duration[lnum][2]-dt_main, pos_start.x(), pos_end.x(), vel_joy.x(), x_dx_ddx_old[lnum]);
                y_dy_ddy[lnum] = FootXY_trajectory(t_temp_leg[lnum], duration[lnum][2]-dt_main, pos_start.y(), pos_end.y(), vel_joy.y(), y_dy_ddy_old[lnum]);
                z_dz_ddz[lnum] = FootZ_Swing_Jump_trajectory(t_temp_leg[lnum], duration[lnum][2]-dt_main, pos_start.z(), pos_end.z(), step_z, z_dz_ddz_old[lnum]);
                state_motion.foot.HL.pos = Vector3d(x_dx_ddx[lnum].x(), y_dy_ddy[lnum].x(), z_dz_ddz[lnum].x());
                state_motion.foot.HL.vel = Vector3d(x_dx_ddx[lnum].y(), y_dy_ddy[lnum].y(), z_dz_ddz[lnum].y());
            }
            if(lnum == FR) {
                // FR swing leg
                Vector3d pos_start =  Endpos_ST[lnum];
                Vector3d pos_end =  QuadState_start.foot.FR.pos + 0.5*pos_step + 0.5*FootStepRotation(pos_rotate,lnum);
                x_dx_ddx[lnum] = FootXY_trajectory(t_temp_leg[lnum], duration[lnum][2]-dt_main, pos_start.x(), pos_end.x(), vel_joy.x(), x_dx_ddx_old[lnum]);
                y_dy_ddy[lnum] = FootXY_trajectory(t_temp_leg[lnum], duration[lnum][2]-dt_main, pos_start.y(), pos_end.y(), vel_joy.y(), y_dy_ddy_old[lnum]);
                z_dz_ddz[lnum] = FootZ_Swing_Jump_trajectory(t_temp_leg[lnum], duration[lnum][2]-dt_main, pos_start.z(), pos_end.z(), step_z, z_dz_ddz_old[lnum]);
                state_motion.foot.FR.pos = Vector3d(x_dx_ddx[lnum].x(), y_dy_ddy[lnum].x(), z_dz_ddz[lnum].x());
                state_motion.foot.FR.vel = Vector3d(x_dx_ddx[lnum].y(), y_dy_ddy[lnum].y(), z_dz_ddz[lnum].y());
            }
            if(lnum == HR){
                // HR, FL stance leg
                Vector3d pos_start =  Endpos_SW[lnum];
                Vector3d pos_end =  QuadState_start.foot.HR.pos - 0.5*pos_step + 0.5*FootStepRotation(-pos_rotate,lnum);
                x_dx_ddx[lnum] = FootXY_trajectory(t_temp_leg[lnum], duration[lnum][2]-dt_main, pos_start.x(), pos_end.x(), vel_joy.x(), x_dx_ddx_old[lnum]);
                y_dy_ddy[lnum] = FootXY_trajectory(t_temp_leg[lnum], duration[lnum][2]-dt_main, pos_start.y(), pos_end.y(), vel_joy.y(), y_dy_ddy_old[lnum]);
                z_dz_ddz[lnum] = FootZ_Stance_Jump_trajectory(t_temp_leg[lnum], duration[lnum][2]-dt_main, pos_start.z(), 1.5, z_dz_ddz_old[lnum]);
                state_motion.foot.HR.pos = Vector3d(x_dx_ddx[lnum].x(), y_dy_ddy[lnum].x(), z_dz_ddz[lnum].x());
                state_motion.foot.HR.vel = Vector3d(x_dx_ddx[lnum].y(), y_dy_ddy[lnum].y(), z_dz_ddz[lnum].y());
            }

            if(lnum == FL){
                // HR, FL stance leg
                Vector3d pos_start =  Endpos_SW[lnum];
                Vector3d pos_end =  QuadState_start.foot.FL.pos - 0.5*pos_step + 0.5*FootStepRotation(-pos_rotate,lnum);
                x_dx_ddx[lnum] = FootXY_trajectory(t_temp_leg[lnum], duration[lnum][2]-dt_main, pos_start.x(), pos_end.x(), vel_joy.x(), x_dx_ddx_old[lnum]);
                y_dy_ddy[lnum] = FootXY_trajectory(t_temp_leg[lnum], duration[lnum][2]-dt_main, pos_start.y(), pos_end.y(), vel_joy.y(), y_dy_ddy_old[lnum]);
                z_dz_ddz[lnum] = FootZ_Stance_Jump_trajectory(t_temp_leg[lnum], duration[lnum][2]-dt_main, pos_start.z(), 1.5, z_dz_ddz_old[lnum]);
                state_motion.foot.FL.pos = Vector3d(x_dx_ddx[lnum].x(), y_dy_ddy[lnum].x(), z_dz_ddz[lnum].x());
                state_motion.foot.FL.vel = Vector3d(x_dx_ddx[lnum].y(), y_dy_ddy[lnum].y(), z_dz_ddz[lnum].y());
            }

            if(t_temp_leg[lnum] >= duration[lnum][2]-dt_main){
                if(lnum == HL){ //swing -> stance
                    Endpos_SW[HL] = state_motion.foot.HL.pos;
                    contact_flag[HL] = true;
                }
                if(lnum == FR){ //swing -> stance
                    Endpos_SW[FR] = state_motion.foot.FR.pos;
                    contact_flag[FR] = true;
                }
                if(lnum == HR){ //stance -> swing
                    Endpos_ST[HR] = state_motion.foot.HR.pos;
                    contact_flag[HR] = false;
                }
                if(lnum == FL){ //stance -> swing
                    Endpos_ST[FL] = state_motion.foot.FL.pos;
                    contact_flag[FL] = false;
                }
            }
            x_dx_ddx_old[lnum] = x_dx_ddx[lnum];
            y_dy_ddy_old[lnum] = y_dy_ddy[lnum];
            z_dz_ddz_old[lnum] = z_dz_ddz[lnum];

            if(start_flag == true) {
                if(t_step_leg[lnum] >= timeline[lnum][3] - dt_main) {
                    t_step_leg[lnum] = timeline[lnum][1] - dt_main;
                    cycle_change_flag = true;
                    step_change_flag = true;

                }
            }
        }
        else if(step_num_leg[lnum] == 3) {
            t_temp_leg[lnum] = t_step_leg[lnum]-timeline[lnum][3];
            //trajectory 3 - ending
            contact_flag[lnum] = true;
        }
        else {
            t_temp_leg[lnum] = t_step_leg[lnum]-timeline[lnum][4];
            contact_flag[lnum] = true;
        }

        if(t_step_leg[lnum] >= interval*num_motion) t_step_leg[lnum] = 0.;

    }
    int sum_step = step_num_leg[0]+step_num_leg[1]+step_num_leg[2]+step_num_leg[3];
    if(start_flag == false && sum_step == num_motion*4) {
        ready_flag = true;
        start_flag = false;
        stop_flag = false;
        mode_state = 0;
        cnt_motion = -1;

        walking_start_flag = true;
        cycle_change_flag = true;
        step_change_flag = true;
    }


    // ------------------------- Position reference output ------------------------
    state_out = state_motion;
//    state_out.foot.HR.pos += state_comp.foot.HR.pos;
//    state_out.foot.HL.pos += state_comp.foot.HL.pos;
//    state_out.foot.FR.pos += state_comp.foot.FR.pos;
//    state_out.foot.FL.pos += state_comp.foot.FL.pos;


    // ------------------------ Task to joint converting --------------------
    QuadJoint JointRef_temp = Kinematics.IK_FullBody(state_out);
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

    //--------------------------------------------------------------------------------------------------
    // COM control parameter

    Vector3d com_Kp_f = Vector3d(sharedData->control_variable_double[1] ,sharedData->control_variable_double[3], sharedData->control_variable_double[5]);
    Vector3d com_Kd_f = Vector3d(sharedData->control_variable_double[2] ,sharedData->control_variable_double[4], sharedData->control_variable_double[6]);
    Matrix3d com_Kp_mat_f = diagonalize(com_Kp_f);
    Matrix3d com_Kd_mat_f = diagonalize(com_Kd_f);

    Vector3d IMU_angle_error = Joy_Orientation - QuadSensor.IMU.angle;
    double angle_error_limit = 10.0*D2Rf;
    Vector3d IMU_angle_err_limited;
    for(int i=0; i<3; i++) {
        if(IMU_angle_error[i] > angle_error_limit) IMU_angle_err_limited[i] = angle_error_limit;
        else if(IMU_angle_error[i] < -angle_error_limit) IMU_angle_err_limited[i] = -angle_error_limit;
        else IMU_angle_err_limited[i] = IMU_angle_error[i];
    }

    com_control_force = com_Kp_mat_f*(IMU_angle_err_limited) + com_Kd_mat_f*-QuadSensor.IMU.vel;

    double limit_f[3] = {800.0, 800.0, 0};
    for(int i=0; i<3; i++) {
        if(com_control_force[i] > limit_f[i]) com_control_force[i] = limit_f[i];
        else if(com_control_force[i] < -limit_f[i]) com_control_force[i] = -limit_f[i];
    }

    foot_Force_COM[HR] = Vector3d(0, 0, +com_control_force.x() -com_control_force.y());
    foot_Force_COM[HL] = Vector3d(0, 0, -com_control_force.x() -com_control_force.y());
    foot_Force_COM[FR] = Vector3d(0, 0, +com_control_force.x() +com_control_force.y());
    foot_Force_COM[FL] = Vector3d(0, 0, -com_control_force.x() +com_control_force.y());


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
                Kp_mat_task[i] = Kp_mat;
                Kd_mat_task[i] = Kd_mat;
                for(int j=0; j<3; j++) {
                    MY_CONTROL_FB_GAIN[i*3+j] = 10;
                }
            }
            else if(contact_flag[i] == true) {
                Kp_mat_task[i] = Kp_mat;
                Kd_mat_task[i] = Kd_mat;
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

    foot_Force_ref[HR] = Kp_mat_task[HR]*(state_out.foot.HR.pos-QuadState.foot.HR.pos) + Kd_mat*(state_out.foot.HR.vel - QuadState_NOIMU.foot.HR.vel);
    foot_Force_ref[HL] = Kp_mat_task[HL]*(state_out.foot.HL.pos-QuadState.foot.HL.pos) + Kd_mat*(state_out.foot.HL.vel - QuadState_NOIMU.foot.HL.vel);
    foot_Force_ref[FR] = Kp_mat_task[FR]*(state_out.foot.FR.pos-QuadState.foot.FR.pos) + Kd_mat*(state_out.foot.FR.vel - QuadState_NOIMU.foot.FR.vel);
    foot_Force_ref[FL] = Kp_mat_task[FL]*(state_out.foot.FL.pos-QuadState.foot.FL.pos) + Kd_mat*(state_out.foot.FL.vel - QuadState_NOIMU.foot.FL.vel);

    // ------------------------ Gravity compensation (contact based) --------------------

    double mg_fz = -sharedData->flyingtrot_variable_double[0];
    double fz_speed = sharedData->flyingtrot_variable_double[1];

    n_contact = contact_flag[HR]+contact_flag[HL]+contact_flag[FR]+contact_flag[FL];

    for(int i=0; i<4; i++) {
        if(contact_flag[i] == true) {
            foot_Force_FF[i].z() -= fz_speed;
            if(n_contact_pre == 4 && n_contact == 2) {
                foot_Force_FF[i].z() = mg_fz/n_contact;
            }
            else {
                if(foot_Force_FF[i].z() < mg_fz/n_contact) foot_Force_FF[i].z() = mg_fz/n_contact;
            }
            foot_Force_COM[i].z() /= n_contact;


            double x_FF_force_gain = sharedData->trot_variable_double[22];
            double y_FF_force_gain = sharedData->trot_variable_double[23];
            double z_FF_force_gain = sharedData->trot_variable_double[24];

            foot_Force_FF[i].x() = vel_joy.x()*x_FF_force_gain + CoM_acc_FF.x()*z_temp;
            foot_Force_FF[i].y() = vel_joy.y()*y_FF_force_gain + CoM_acc_FF.y()*z_FF_force_gain;

        }
        else {
            foot_Force_COM[i].z() = 0;
            foot_Force_FF[i].z() = 0;
            foot_Force_FF[i].x() = 0;
            foot_Force_FF[i].y() = 0;
        }
    }


    n_contact_pre = n_contact;

    // ------------------------ Feedforward force contraints --------------------
    Vector3d FF_temp[4];
    for(int i=0 ; i<4 ; i++){
        FF_temp[i] = foot_Force_FF[i] + foot_Force_COM[i];
        if(FF_temp[i].z() > 30)  FF_temp[i].z() = 30;
    }

    // ------------------------ SUM feedforward force --------------------
    QuadStateRef.foot.HR.force = foot_Force_ref[HR] + FF_temp[HR];
    QuadStateRef.foot.HL.force = foot_Force_ref[HL] + FF_temp[HL];
    QuadStateRef.foot.FR.force = foot_Force_ref[FR] + FF_temp[FR];
    QuadStateRef.foot.FL.force = foot_Force_ref[FL] + FF_temp[FL];

    // ------------------------ Force to Torque converting --------------------

    foot_Torque_ref[HR] = legJacobian[HR].transpose()*QuadStateRef.foot.HR.force;
    foot_Torque_ref[HL] = legJacobian[HL].transpose()*QuadStateRef.foot.HL.force;
    foot_Torque_ref[FR] = legJacobian[FR].transpose()*QuadStateRef.foot.FR.force;
    foot_Torque_ref[FL] = legJacobian[FL].transpose()*QuadStateRef.foot.FL.force;

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

    current_limit_ = 30.0;

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
                MY_CONTROL_FF_GAIN[i*3+j] = 10;
            }
        }
        else {
            for(int j=0; j<3; j++) {
                MY_CONTROL_FF_GAIN[i*3+j] = 0;
            }
        }
    }

    sharedData->contact_flag[HR] = contact_flag[HR];
    sharedData->contact_flag[HL] = contact_flag[HL];
    sharedData->contact_flag[FR] = contact_flag[FR];
    sharedData->contact_flag[FL] = contact_flag[FL];


    if(sharedData->isFalled == true) {
        cout << "Motion Falled." << endl;
        task_idx = NoTask;
//        sharedData->plot_play_ON = false;
        isMotionEnabled = false;
        cnt_motion = -1;

        sharedData->NEWCOMMAND = true;
        sharedData->COMMAND.USER_COMMAND = MOTION_FALLING;

        task_start_flag = true;
        walking_start_flag = true;
        cycle_change_flag = true;
        step_change_flag = true;

        JointRef = QuadSensorRef.Encoder.pos;
        return JointRef;
    }

    if(sharedData->isStopping == true) {
        cout << "Motion Finished." << endl;
        task_idx = NoTask;
//        sharedData->plot_play_ON = false;
        sharedData->incGain_flag = false;
        isMotionEnabled = false;
        cnt_motion = -1;

        task_start_flag = true;
        walking_start_flag = true;
        cycle_change_flag = true;
        step_change_flag = true;

        JointRef = QuadSensorRef.Encoder.pos;
        return JointRef;
    }

    if((sharedData->COMMAND.USER_COMMAND != MOTION_STOP && ready_flag != true))
        cnt_motion++;

    sharedData->comcon_variable_plot[0] = ComState.vel.x();
    sharedData->comcon_variable_plot[1] = ComState.vel.y();
    sharedData->comcon_variable_plot[2] = ComState.pos.x();
    sharedData->comcon_variable_plot[3] = ComState.pos.y();
    sharedData->comcon_variable_plot[4] = ComStateRef.vel.x();
    sharedData->comcon_variable_plot[5] = ComStateRef.vel.y();


    //printf("%d, %d, %d, %d\n", contact_flag[0],contact_flag[1],contact_flag[2],contact_flag[3]);

    //printf("%.3f, %.3f, %.3f, %.3f\n", foot_Force_FF[0].z(),foot_Force_FF[1].z(),foot_Force_FF[2].z(),foot_Force_FF[3].z());

//    printf("t : %.3f, step : %d, %d,  t: %.3f, %.3f,  t: %.3f, %.3f\n", t_sim, step_num_leg[HR], step_num_leg[HL], t_temp_leg[HR], t_temp_leg[HL], t_step_leg[HR], t_step_leg[HL]);

    return  JointRef;
}
