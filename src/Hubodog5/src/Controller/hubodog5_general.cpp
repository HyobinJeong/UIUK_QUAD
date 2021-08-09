
#include "hubodog5_general.h"

extern RBMotorController   _DEV_MC[MAX_MC];
extern RBCAN    *canHandler;
extern pRBCORE_SHM sharedData;

extern int MY_CONTROL_FB_GAIN[12];
extern int MY_CONTROL_FF_GAIN[12];

extern SensorInfo sensor;
extern hubodog5_general Quad;

void hubodog5_general::initialize()
{
    FILE_LOG(logSUCCESS) << "Hubodog5 Parameter Initialize.";
    cnt_motion = 0;
    t_sim = 0.0;
    z0_start = sharedData->z0;
    x0_start = sharedData->x0;
    QuadState_readypos = Kinematics.FK_COM(QuadSensorRef, Vector3d(x0_start,0,z0_start));
    ComState.pos = QuadState_readypos.COM.pos;

    Contact_point = ComState.pos;
    Contact_point_ref = ComState.pos;

    Contact_point.z() = 0.0;
    dt_main = 0.002;

    // uiuk init
    ready_flag = false;
    start_flag = false;
    stop_flag = false;
    onestep_flag = false;

    landing_flag[HR] = false;
    landing_flag[FL] = false;
    landing_flag[HL] = false;
    landing_flag[FR] = false;

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

    for(int i=0; i<4; i++){
        foot_Force_FF[i] = Vector3dZero;

        foot_Force_COM[i] = Vector3dZero;

        foot_Force_ref[i] = Vector3dZero;

        com_control_force = Vector3dZero;

        vel_joy = Vector3dZero;
        acc_joy = Vector3dZero;

        ComState.vel = Vector3dZero;
        ComStateRef.vel = Vector3dZero;
    }

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
    end_flag = false;
    onestep_flag = false;
    mode_state = 0;

}

void hubodog5_general::motion_start()
{
//    FILE_LOG(logSUCCESS) << "MOTION START";
    task_idx = sharedData->COMMAND.USER_PARA_CHAR[0];
    isMotionEnabled = true;
    isPosAdjust = true;
    isReadyDone = false;
}

void hubodog5_general::motion_stop()
{
//    FILE_LOG(logSUCCESS) << "MOTION STOP";
    isMotionStopping = true;
//    isPosAdjust = true;
}




QuadJoint hubodog5_general::pos_adjust()
{
    QuadJoint JointRef;
    double adjust_T = 1.0;

    // save check
    if(sharedData->isStopping)
    {
//        if(isSavingData)
//            save_file();
        cnt_motion = 0;
        t_sim = 0;
//        sharedData->isStopping = false;
    }



    if(cnt_motion < 1)
    {
        cout << "Adjust Initial Posture." << endl;
//        QuadSensor_start = QuadSensor;
        PosAdjust_init = QuadSensorRef.Encoder.pos; // confim

//        cout << PosAdjust_init.HRR * R2Df << endl;
//        cout << PosAdjust_init.HRP * R2Df << endl;
//        cout << PosAdjust_init.HRK * R2Df << endl;
//        RobotState FK_temp = Kinematics.FK_FullBody(QuadSensor);

        ALLPCON();

        RobotState state_temp = QuadState_readypos;
        state_temp.foot.HR.pos[0] = state_temp.foot.HR.pos[0] - x0_start;
        state_temp.foot.HL.pos[0] = state_temp.foot.HL.pos[0] - x0_start;
        state_temp.foot.FR.pos[0] = state_temp.foot.FR.pos[0] - x0_start;
        state_temp.foot.FL.pos[0] = state_temp.foot.FL.pos[0] - x0_start;
        state_temp.foot.HR.pos[2] = 0;
        state_temp.foot.HL.pos[2] = 0;
        state_temp.foot.FR.pos[2] = 0;
        state_temp.foot.FL.pos[2] = 0;

//        cout << "COM = " << state_temp.COM.pos << endl;
//        cout << "R = " << state_temp.rotate << endl;
//        cout << "pf_HR = " << state_temp.foot.HR.pos << endl;
//        cout << "pf_HL = " << state_temp.foot.HL.pos << endl;
//        cout << "pf_FR = " << state_temp.foot.FR.pos << endl;
//        cout << "pf_FL = " << state_temp.foot.FL.pos << endl;
        PosAdjust_final = Kinematics.IK_FullBody(state_temp);
    }

    if(t_sim > adjust_T) {t_sim = adjust_T;}

    Vector3d ref_temp, init_now, final_temp;

    ref_temp = bezier_calc(t_sim, adjust_T, Vector3d(PosAdjust_init.HRR,  0,  0), Vector3d(PosAdjust_final.HRR, 0, 0));
    JointRef.HRR = ref_temp[0];
    ref_temp = bezier_calc(t_sim, adjust_T, Vector3d(PosAdjust_init.HRP,  0,  0), Vector3d(PosAdjust_final.HRP, 0, 0));
    JointRef.HRP = ref_temp[0];
    ref_temp = bezier_calc(t_sim, adjust_T, Vector3d(PosAdjust_init.HRK,  0,  0), Vector3d(PosAdjust_final.HRK, 0, 0));
    JointRef.HRK = ref_temp[0];

    ref_temp = bezier_calc(t_sim, adjust_T, Vector3d(PosAdjust_init.HLR,  0,  0), Vector3d(PosAdjust_final.HLR, 0, 0));
    JointRef.HLR = ref_temp[0];
    ref_temp = bezier_calc(t_sim, adjust_T, Vector3d(PosAdjust_init.HLP,  0,  0), Vector3d(PosAdjust_final.HLP, 0, 0));
    JointRef.HLP = ref_temp[0];
    ref_temp = bezier_calc(t_sim, adjust_T, Vector3d(PosAdjust_init.HLK,  0,  0), Vector3d(PosAdjust_final.HLK, 0, 0));
    JointRef.HLK = ref_temp[0];

    ref_temp = bezier_calc(t_sim, adjust_T, Vector3d(PosAdjust_init.FRR,  0,  0), Vector3d(PosAdjust_final.FRR, 0, 0));
    JointRef.FRR = ref_temp[0];
    ref_temp = bezier_calc(t_sim, adjust_T, Vector3d(PosAdjust_init.FRP,  0,  0), Vector3d(PosAdjust_final.FRP, 0, 0));
    JointRef.FRP = ref_temp[0];
    ref_temp = bezier_calc(t_sim, adjust_T, Vector3d(PosAdjust_init.FRK,  0,  0), Vector3d(PosAdjust_final.FRK, 0, 0));
    JointRef.FRK = ref_temp[0];

    ref_temp = bezier_calc(t_sim, adjust_T, Vector3d(PosAdjust_init.FLR,  0,  0), Vector3d(PosAdjust_final.FLR, 0, 0));
    JointRef.FLR = ref_temp[0];
    ref_temp = bezier_calc(t_sim, adjust_T, Vector3d(PosAdjust_init.FLP,  0,  0), Vector3d(PosAdjust_final.FLP, 0, 0));
    JointRef.FLP = ref_temp[0];
    ref_temp = bezier_calc(t_sim, adjust_T, Vector3d(PosAdjust_init.FLK,  0,  0), Vector3d(PosAdjust_final.FLK, 0, 0));
    JointRef.FLK = ref_temp[0];
//    cout << JointRef.FLK * R2Df << endl;


    if(t_sim >= adjust_T)
    {
        cout << "Adjust Done. ";
        isPosAdjust = false;
        cnt_motion = -1;
//        sharedData->plot_timer_zero = true;
//        sharedData->plot_play_ON = true;

        if(sharedData->COMMAND.USER_COMMAND == MOTION_STOP)
        {
            cout << "Motion Finished." << endl;
            sharedData->COMMAND.USER_COMMAND = NO_ACT;
//            sharedData->plot_play_ON = false;
            isMotionEnabled = false;
            task_idx = NoTask;
        }
        else{
            cout << endl;
        }

        QuadState_start = Kinematics.FK_COM(QuadSensorRef, Vector3d(x0_start,0,z0_start));
        QuadState_start.foot.HR.vel = Vector3dZero;
        QuadState_start.foot.HL.vel = Vector3dZero;
        QuadState_start.foot.FR.vel = Vector3dZero;
        QuadState_start.foot.FL.vel = Vector3dZero;

        if(sharedData->savedata_ON)
        {
            sharedData->savedata_ON = false;
            isSavingData = true;
        }
        return JointRef;
    }

    cnt_motion++;

//    JointRef = QuadSensor.Encoder.pos;
    return  JointRef;
}

void hubodog5_general::ALLPCON()
{
//    FILE_LOG(logSUCCESS) <<"ALL PCON";
    for(int i=0; i<NO_OF_JOINTS; i++) {Pcon[i] = true;}
}

void hubodog5_general::ALLCCON()
{
//    FILE_LOG(logSUCCESS) <<"ALL CCON";
    for(int i=0; i<NO_OF_JOINTS; i++) {Ccon[i] = true;}
}

void hubodog5_general::ALLPCCON()
{
//    FILE_LOG(logSUCCESS) <<"ALL PCCON";
    for(int i=0; i<NO_OF_JOINTS; i++) {PCcon[i] = true;}
}

void hubodog5_general::save_onestep(int cnt)
{
    if(cnt<SaveMaxCnt)
    {
        // sensor value
        SAVE[idx_JointPosEnc+0][cnt]    = QuadSensor.Encoder.pos.HRR;
        SAVE[idx_JointPosEnc+1][cnt]    = QuadSensor.Encoder.pos.HRP;
        SAVE[idx_JointPosEnc+2][cnt]    = QuadSensor.Encoder.pos.HRK;
        SAVE[idx_JointPosEnc+3][cnt]    = QuadSensor.Encoder.pos.HLR;
        SAVE[idx_JointPosEnc+4][cnt]    = QuadSensor.Encoder.pos.HLP;
        SAVE[idx_JointPosEnc+5][cnt]    = QuadSensor.Encoder.pos.HLK;
        SAVE[idx_JointPosEnc+6][cnt]    = QuadSensor.Encoder.pos.FRR;
        SAVE[idx_JointPosEnc+7][cnt]    = QuadSensor.Encoder.pos.FRP;
        SAVE[idx_JointPosEnc+8][cnt]    = QuadSensor.Encoder.pos.FRK;
        SAVE[idx_JointPosEnc+9][cnt]    = QuadSensor.Encoder.pos.FLR;
        SAVE[idx_JointPosEnc+10][cnt]   = QuadSensor.Encoder.pos.FLP;
        SAVE[idx_JointPosEnc+11][cnt]   = QuadSensor.Encoder.pos.FLK;

        SAVE[idx_JointVel+0][cnt]   = QuadSensor.Encoder.vel.HRR;
        SAVE[idx_JointVel+1][cnt]   = QuadSensor.Encoder.vel.HRP;
        SAVE[idx_JointVel+2][cnt]   = QuadSensor.Encoder.vel.HRK;
        SAVE[idx_JointVel+3][cnt]   = QuadSensor.Encoder.vel.HLR;
        SAVE[idx_JointVel+4][cnt]   = QuadSensor.Encoder.vel.HLP;
        SAVE[idx_JointVel+5][cnt]   = QuadSensor.Encoder.vel.HLK;
        SAVE[idx_JointVel+6][cnt]   = QuadSensor.Encoder.vel.FRR;
        SAVE[idx_JointVel+7][cnt]   = QuadSensor.Encoder.vel.FRP;
        SAVE[idx_JointVel+8][cnt]   = QuadSensor.Encoder.vel.FRK;
        SAVE[idx_JointVel+9][cnt]   = QuadSensor.Encoder.vel.FLR;
        SAVE[idx_JointVel+10][cnt]  = QuadSensor.Encoder.vel.FLP;
        SAVE[idx_JointVel+11][cnt]  = QuadSensor.Encoder.vel.FLK;

        SAVE[idx_JointCurrent+0][cnt]   = QuadSensor.Encoder.current.HRR;
        SAVE[idx_JointCurrent+1][cnt]   = QuadSensor.Encoder.current.HRP;
        SAVE[idx_JointCurrent+2][cnt]   = QuadSensor.Encoder.current.HRK;
        SAVE[idx_JointCurrent+3][cnt]   = QuadSensor.Encoder.current.HLR;
        SAVE[idx_JointCurrent+4][cnt]   = QuadSensor.Encoder.current.HLP;
        SAVE[idx_JointCurrent+5][cnt]   = QuadSensor.Encoder.current.HLK;
        SAVE[idx_JointCurrent+6][cnt]   = QuadSensor.Encoder.current.FRR;
        SAVE[idx_JointCurrent+7][cnt]   = QuadSensor.Encoder.current.FRP;
        SAVE[idx_JointCurrent+8][cnt]   = QuadSensor.Encoder.current.FRK;
        SAVE[idx_JointCurrent+9][cnt]   = QuadSensor.Encoder.current.FLR;
        SAVE[idx_JointCurrent+10][cnt]  = QuadSensor.Encoder.current.FLP;
        SAVE[idx_JointCurrent+11][cnt]  = QuadSensor.Encoder.current.FLK;

        SAVE[idx_JointPosEnc_filt+0][cnt]   = QuadSensor_filtered.Encoder.pos.HRR;
        SAVE[idx_JointPosEnc_filt+1][cnt]   = QuadSensor_filtered.Encoder.pos.HRP;
        SAVE[idx_JointPosEnc_filt+2][cnt]   = QuadSensor_filtered.Encoder.pos.HRK;
        SAVE[idx_JointPosEnc_filt+3][cnt]   = QuadSensor_filtered.Encoder.pos.HLR;
        SAVE[idx_JointPosEnc_filt+4][cnt]   = QuadSensor_filtered.Encoder.pos.HLP;
        SAVE[idx_JointPosEnc_filt+5][cnt]   = QuadSensor_filtered.Encoder.pos.HLK;
        SAVE[idx_JointPosEnc_filt+6][cnt]   = QuadSensor_filtered.Encoder.pos.FRR;
        SAVE[idx_JointPosEnc_filt+7][cnt]   = QuadSensor_filtered.Encoder.pos.FRP;
        SAVE[idx_JointPosEnc_filt+8][cnt]   = QuadSensor_filtered.Encoder.pos.FRK;
        SAVE[idx_JointPosEnc_filt+9][cnt]   = QuadSensor_filtered.Encoder.pos.FLR;
        SAVE[idx_JointPosEnc_filt+10][cnt]  = QuadSensor_filtered.Encoder.pos.FLP;
        SAVE[idx_JointPosEnc_filt+11][cnt]  = QuadSensor_filtered.Encoder.pos.FLK;

        SAVE[idx_JointVel_filt+0][cnt]  = QuadSensor_filtered.Encoder.vel.HRR;
        SAVE[idx_JointVel_filt+1][cnt]  = QuadSensor_filtered.Encoder.vel.HRP;
        SAVE[idx_JointVel_filt+2][cnt]  = QuadSensor_filtered.Encoder.vel.HRK;
        SAVE[idx_JointVel_filt+3][cnt]  = QuadSensor_filtered.Encoder.vel.HLR;
        SAVE[idx_JointVel_filt+4][cnt]  = QuadSensor_filtered.Encoder.vel.HLP;
        SAVE[idx_JointVel_filt+5][cnt]  = QuadSensor_filtered.Encoder.vel.HLK;
        SAVE[idx_JointVel_filt+6][cnt]  = QuadSensor_filtered.Encoder.vel.FRR;
        SAVE[idx_JointVel_filt+7][cnt]  = QuadSensor_filtered.Encoder.vel.FRP;
        SAVE[idx_JointVel_filt+8][cnt]  = QuadSensor_filtered.Encoder.vel.FRK;
        SAVE[idx_JointVel_filt+9][cnt]  = QuadSensor_filtered.Encoder.vel.FLR;
        SAVE[idx_JointVel_filt+10][cnt] = QuadSensor_filtered.Encoder.vel.FLP;
        SAVE[idx_JointVel_filt+11][cnt] = QuadSensor_filtered.Encoder.vel.FLK;

        SAVE[idx_JointCurrent_filt+0][cnt]  = QuadSensor_filtered.Encoder.current.HRR;
        SAVE[idx_JointCurrent_filt+1][cnt]  = QuadSensor_filtered.Encoder.current.HRP;
        SAVE[idx_JointCurrent_filt+2][cnt]  = QuadSensor_filtered.Encoder.current.HRK;
        SAVE[idx_JointCurrent_filt+3][cnt]  = QuadSensor_filtered.Encoder.current.HLR;
        SAVE[idx_JointCurrent_filt+4][cnt]  = QuadSensor_filtered.Encoder.current.HLP;
        SAVE[idx_JointCurrent_filt+5][cnt]  = QuadSensor_filtered.Encoder.current.HLK;
        SAVE[idx_JointCurrent_filt+6][cnt]  = QuadSensor_filtered.Encoder.current.FRR;
        SAVE[idx_JointCurrent_filt+7][cnt]  = QuadSensor_filtered.Encoder.current.FRP;
        SAVE[idx_JointCurrent_filt+8][cnt]  = QuadSensor_filtered.Encoder.current.FRK;
        SAVE[idx_JointCurrent_filt+9][cnt]  = QuadSensor_filtered.Encoder.current.FLR;
        SAVE[idx_JointCurrent_filt+10][cnt] = QuadSensor_filtered.Encoder.current.FLP;
        SAVE[idx_JointCurrent_filt+11][cnt] = QuadSensor_filtered.Encoder.current.FLK;

        for(int i=0; i<3; i++)
        {
            SAVE[idx_IMUangle+i][cnt]       = QuadSensor.IMU.angle[i];
            SAVE[idx_IMUquat+i][cnt]        = QuadSensor.IMU.quat[i];
            SAVE[idx_IMUomega_raw+i][cnt]   = QuadSensor.IMU.vel[i];
            SAVE[idx_IMUomega+i][cnt]       = QuadSensor_filtered.IMU.vel[i];
            SAVE[idx_IMUacc+i][cnt]         = QuadSensor.IMU.acc[i];
        }
        SAVE[idx_IMUquat+3][cnt] = QuadSensor.IMU.quat[3];

        // MPC state & control input & reference
        for(int i=0; i<24; i++)
        {
            SAVE[idx_xmeasured+i][cnt] = x_measured[i];
            SAVE[idx_xcurrent+i][cnt] = x[i];
            SAVE[idx_xref+i][cnt] = xs[i];
        }
        SAVE[idx_xcurrent+6][cnt] = rpy_estimated[0];
        SAVE[idx_xcurrent+7][cnt] = rpy_estimated[1];
        SAVE[idx_xcurrent+8][cnt] = rpy_estimated[2];
        for(int i=0; i<12; i++)
        {
            SAVE[idx_GRFRef+i][cnt] = uref[i];
        }
        SAVE[idx_TorqueRef+0][cnt]  = TorqueRef.HRR;
        SAVE[idx_TorqueRef+1][cnt]  = TorqueRef.HRP;
        SAVE[idx_TorqueRef+2][cnt]  = TorqueRef.HRK;
        SAVE[idx_TorqueRef+3][cnt]  = TorqueRef.HLR;
        SAVE[idx_TorqueRef+4][cnt]  = TorqueRef.HLP;
        SAVE[idx_TorqueRef+5][cnt]  = TorqueRef.HLK;
        SAVE[idx_TorqueRef+6][cnt]  = TorqueRef.FRR;
        SAVE[idx_TorqueRef+7][cnt]  = TorqueRef.FRP;
        SAVE[idx_TorqueRef+8][cnt]  = TorqueRef.FRK;
        SAVE[idx_TorqueRef+9][cnt]  = TorqueRef.FLR;
        SAVE[idx_TorqueRef+10][cnt] = TorqueRef.FLP;
        SAVE[idx_TorqueRef+11][cnt] = TorqueRef.FLK;

        // MPC parameters
        for(int i=0; i<24+12+1; i++){SAVE[idx_weight+i][cnt] = weight_mpc[i];}
        SAVE[idx_Nhorizon+0][cnt] = Nhorizon;
        SAVE[idx_Tstep+0][cnt] = Tstep;
        SAVE[idx_Ts_mpc+0][cnt] = Ts_mpc;
        SAVE[idx_mu+0][cnt] = mu;
        SAVE[idx_Fzmax+0][cnt] = Fz_max;
//        SAVE[idx_mpcFrq_solve+0][cnt] = sharedData->mpc_tsolve;
//        SAVE[idx_mpcFrq_total+0][cnt] = sharedData->mpc_timer_total;
//        SAVE[idx_mpcIter+0][cnt] = sharedData->mpc_iter;
//        SAVE[idx_x_junny+0][cnt] = sharedData->X_estimated.Position[0];
//        SAVE[idx_x_junny+1][cnt] = sharedData->X_estimated.Position[1];
//        SAVE[idx_x_junny+2][cnt] = sharedData->X_estimated.Position[2];
//        SAVE[idx_x_junny+3][cnt] = sharedData->X_estimated.Velocity[0];
//        SAVE[idx_x_junny+4][cnt] = sharedData->X_estimated.Velocity[1];
//        SAVE[idx_x_junny+5][cnt] = sharedData->X_estimated.Velocity[2];
//        SAVE[idx_x_junny+6][cnt] = sharedData->X_estimated.rpy[0];
//        SAVE[idx_x_junny+7][cnt] = sharedData->X_estimated.rpy[1];
//        SAVE[idx_x_junny+8][cnt] = sharedData->X_estimated.rpy[2];
//        SAVE[idx_x_junny+9][cnt] = sharedData->X_estimated.Angvel[0];
//        SAVE[idx_x_junny+10][cnt] =sharedData->X_estimated.Angvel[1];
//        SAVE[idx_x_junny+11][cnt] =sharedData->X_estimated.Angvel[2];
        for(int i=0; i<4; i++)
        {
            SAVE[idx_contactflag+i][cnt] = flag_contact[i];
            SAVE[idx_contactflag_ref+i][cnt] = gait_current[i];
        }

        for(int i=0; i<12; i++)
        {
            //SAVE[idx_extForce+i][cnt] = sharedData->extForce[i];
        }
    }
}

void hubodog5_general::save_file()
{
    if(!isSavingData || cnt_motion < 100)
    {
        FILE_LOG(logINFO) << "NO SAVE";
        return;
    }
    FILE_LOG(logINFO) << "Save Data File.";
    isSavingData = false;

    FILE* ffp = NULL;
    std::string str("../log/");
    std::time_t rawtime;
    std::tm* timeinfo;
    char buffer[80];
    std::time(&rawtime);
    timeinfo = std::localtime(&rawtime);
//    std::strftime(buffer,80,"%Y%m%d%H%M%S",timeinfo);
    std::strftime(buffer,80,"%Y-%m-%d-%H-%M-%S",timeinfo);
    std::string timestr(buffer);

//    std::ostringstream strs,strs2,strs3;
    str = str + timestr;
    if(isSimulation)
        str = str + "_SIM";
    str = str + "_" + TaskName;
    if(*sharedData->CustomFileName != NULL)
        str = str + "_" + sharedData->CustomFileName;

//    strs << Nhorizon;
//    str = str + "_N=" + strs.str();
//    strs2 << Ts_mpc;
//    str = str + "_Ts=" + strs2.str();
//    strs3 << Tstep;
//    str = str + "_Tstep=" + strs3.str();
    str = str + ".txt";
    ffp= fopen(str.c_str(),"w");
    int savenum = min(cnt_motion, SaveMaxCnt);
    for(int i=0;i<savenum;i++)
    {
        for(int j=0;j<SaveMaxData;j++)
        {
            fprintf(ffp,"%f\t",SAVE[j][i]);
        }
        fprintf(ffp,"\n");
        if(i%10000==0)
        {
            printf("saving... %d / %d\n",i,cnt_motion);
        }
    }

    fclose(ffp);
    printf("%s\n",str.c_str());
    FILE_LOG(logSUCCESS) << "Save Done.";
}


