#include "hubodog5_general.h"
#include "func_sensor.h"
#include "RBSharedMemory.h"

extern RBMotorController   _DEV_MC[MAX_MC];
extern int MY_CONTROL_FB_GAIN[12];
extern int MY_CONTROL_FF_GAIN[12];

int hopping_init_flag = true;
int hopping_mode_state = 0;
int hopping_mode_state_pre = 0;

QuadJoint hubodog5_general::task_hopping_test() {
    QuadJoint JointRef;

    int num_motion = 5;
    double interval_temp[5];
    double interval_pause_temp[5];
    double t_temp;
    double interval;
    double interval_pause;
    double pos_z[5];
    double pos_x = 0.05;
    double pos_z_ofs = -0.015;
    //double pos_z_ofs = 0;

    double timeline[num_motion*2+2]; timeline[0] = 0.0;

    //down - push - fold - landing - recover
    for(int i=0; i<5; i++) {
        interval_temp[i] = sharedData->hopping_variable_double[i];
        pos_z[i] = sharedData->hopping_variable_double[6+i];
    }

    for(int i=0; i<num_motion; i++) {

        interval_pause_temp[i] = sharedData->hopping_variable_double[5];

        timeline[i*2+1] = timeline[i*2] + interval_temp[i];
        timeline[i*2+2] = timeline[i*2+1] + interval_pause_temp[i];
    }

    mode_state_task_pre = mode_state_task;
    mode_state_task = sharedData->hopping_variable_int[0];
    if(mode_state_task != mode_state_task_pre) mode_state = mode_state_task;

//    joy_state_task_pre = joy_state_task;
//    joy_state_task = sharedData->joy_state;
//    if(joy_state_task != joy_state_task_pre) mode_state = joy_state_task;

    ready_flag = false;
    start_flag = false;
    stop_flag = false;
    onestep_flag = false;

    if(mode_state == READY)
        ready_flag = true;
    else if(mode_state == START)
        start_flag = true;
    else if(mode_state == STOP)
        stop_flag = true;
    else if(mode_state == ONESTEP)
        onestep_flag = true;


    if(hopping_init_flag == true) {

        if(start_flag == true) {
            cout << "Hopping Test" << endl;
            TaskName = "Hopping";
        }

        int kp_r = sharedData->Kp_j[0];
        int kp_p = sharedData->Kp_j[1];
        int kp_k = sharedData->Kp_j[2];
        int kd_r = sharedData->Kd_j[0];
        int kd_p = sharedData->Kd_j[1];
        int kd_k = sharedData->Kd_j[2];

        _DEV_MC[HRR].RBFOC_SetGain_POS(kp_r,0,kd_r, 1);
        _DEV_MC[HRP].RBFOC_SetGain_POS(kp_p,0,kd_p, 1);
        _DEV_MC[HRK].RBFOC_SetGain_POS(kp_k,0,kd_k, 1);

        _DEV_MC[HLR].RBFOC_SetGain_POS(kp_r,0,kd_r, 1);
        _DEV_MC[HLP].RBFOC_SetGain_POS(kp_p,0,kd_p, 1);
        _DEV_MC[HLK].RBFOC_SetGain_POS(kp_k,0,kd_k, 1);

        _DEV_MC[FRR].RBFOC_SetGain_POS(kp_r,0,kd_r, 1);
        _DEV_MC[FRP].RBFOC_SetGain_POS(kp_p,0,kd_p, 1);
        _DEV_MC[FRK].RBFOC_SetGain_POS(kp_k,0,kd_k, 1);

        _DEV_MC[FLR].RBFOC_SetGain_POS(kp_r,0,kd_r, 1);
        _DEV_MC[FLP].RBFOC_SetGain_POS(kp_p,0,kd_p, 1);
        _DEV_MC[FLK].RBFOC_SetGain_POS(kp_k,0,kd_k, 1);

        hopping_init_flag = false;
    }

    Joy_X = Lowpass_filter(sharedData->joy_dpCOM[0]*0.4, Joy_X, 1.0, dt_main);
    Joy_Y = Lowpass_filter(sharedData->joy_dpCOM[1]*0.8, Joy_Y, 1.0, dt_main);
    Joy_R =  Lowpass_filter(sharedData->joy_dRPY[2]*0.4, Joy_R, 1.0, dt_main);

    Joy_step = Vector3d(Joy_X, Joy_Y, 0);
    Joy_rotation = -Joy_R;

    pos_step = Joy_step;
    pos_rotate = Joy_rotation;

    if(t_sim <= 0)                                   step_num = -1;
    else if(t_sim<timeline[1])                       step_num = 0;
    else if(t_sim>=timeline[1] && t_sim<timeline[2]) step_num = 1;
    else if(t_sim>=timeline[2] && t_sim<timeline[3]) step_num = 2;
    else if(t_sim>=timeline[3] && t_sim<timeline[4]) step_num = 3;
    else if(t_sim>=timeline[4] && t_sim<timeline[5]) step_num = 4;
    else if(t_sim>=timeline[5] && t_sim<timeline[6]) step_num = 5;
    else if(t_sim>=timeline[6] && t_sim<timeline[7]) step_num = 6;
    else if(t_sim>=timeline[7] && t_sim<timeline[8]) step_num = 7;
    else if(t_sim>=timeline[8] && t_sim<timeline[9]) step_num = 8;
    else if(t_sim>=timeline[9] && t_sim<timeline[10]) step_num = 9;
    else                                             step_num = 10;

    if(step_num == -1) {
        state_motion = QuadState_start;
    }
    else if(step_num == 0){ // ------------------ motion 00 --------------------- down
        t_temp = t_sim-timeline[step_num];
        interval = interval_temp[step_num/2];

        state_motion.foot.HR.pos = MovePos(t_temp, interval,
                                           QuadState_start.foot.HR.pos,
                                           QuadState_start.foot.HR.pos + Vector3d(0,0,pos_z[0]));
        state_motion.foot.HL.pos = MovePos(t_temp, interval,
                                           QuadState_start.foot.HL.pos,
                                           QuadState_start.foot.HL.pos + Vector3d(0,0,pos_z[0]));
        state_motion.foot.FR.pos = MovePos(t_temp, interval,
                                           QuadState_start.foot.FR.pos,
                                           QuadState_start.foot.FR.pos + Vector3d(0,0,pos_z[0]));
        state_motion.foot.FL.pos = MovePos(t_temp, interval,
                                           QuadState_start.foot.FL.pos,
                                           QuadState_start.foot.FL.pos + Vector3d(0,0,pos_z[0]));

    }

    else if(step_num == 1){

    } //------------------------------------PAUSE--------------------------------------------------

    else if(step_num == 2){ // ------------------ motion 01 --------------------- push
        t_temp = t_sim-timeline[step_num];
        interval = interval_temp[step_num/2];

//        state_motion.foot.HR.pos = MovePos(t_temp, interval,
//                                           QuadState_start.foot.HR.pos + Vector3d(0,0,pos_z[0]),
//                                           QuadState_start.foot.HR.pos + Vector3d(0,0,pos_z[1]));
//        state_motion.foot.HL.pos = MovePos(t_temp, interval,
//                                           QuadState_start.foot.HL.pos + Vector3d(0,0,pos_z[0]),
//                                           QuadState_start.foot.HL.pos + Vector3d(0,0,pos_z[1]));
//        state_motion.foot.FR.pos = MovePos(t_temp, interval,
//                                           QuadState_start.foot.FR.pos + Vector3d(0,0,pos_z[0]),
//                                           QuadState_start.foot.FR.pos + Vector3d(0,0,pos_z[1]));
//        state_motion.foot.FL.pos = MovePos(t_temp, interval,
//                                           QuadState_start.foot.FL.pos + Vector3d(0,0,pos_z[0]),
//                                           QuadState_start.foot.FL.pos + Vector3d(0,0,pos_z[1]));

        state_motion.foot.HR.pos = MovePos(t_temp, interval,
                                           QuadState_start.foot.HR.pos + Vector3d(0,0,0),
                                           QuadState_start.foot.HR.pos + Vector3d(0,0,pos_z[1]+pos_z_ofs) - 0.5*(pos_step+FootStepRotation(pos_rotate,HR)));
        state_motion.foot.HL.pos = MovePos(t_temp, interval,
                                           QuadState_start.foot.HL.pos + Vector3d(0,0,0),
                                           QuadState_start.foot.HL.pos + Vector3d(0,0,pos_z[1]+pos_z_ofs) - 0.5*(pos_step+FootStepRotation(pos_rotate,HL)));
        state_motion.foot.FR.pos = MovePos(t_temp, interval,
                                           QuadState_start.foot.FR.pos + Vector3d(0,0,0),
                                           QuadState_start.foot.FR.pos + Vector3d(0,0,pos_z[1]) - 0.5*(pos_step+FootStepRotation(pos_rotate,FR)));
        state_motion.foot.FL.pos = MovePos(t_temp, interval,
                                           QuadState_start.foot.FL.pos + Vector3d(0,0,0),
                                           QuadState_start.foot.FL.pos + Vector3d(0,0,pos_z[1]) - 0.5*(pos_step+FootStepRotation(pos_rotate,FL)));

    }

    else if(step_num == 3){
    } //------------------------------------PAUSE--------------------------------------------------

    else if(step_num == 4){ // ------------------ motion 03 --------------------- fold
        t_temp = t_sim-timeline[step_num];
        interval = interval_temp[step_num/2];

//        state_motion.foot.HR.pos = MovePos(t_temp, interval,
//                                           QuadState_start.foot.HR.pos + Vector3d(0,0,pos_z[1]),
//                                           QuadState_start.foot.HR.pos + Vector3d(0,0,pos_z[2]));
//        state_motion.foot.HL.pos = MovePos(t_temp, interval,
//                                           QuadState_start.foot.HL.pos + Vector3d(0,0,pos_z[1]),
//                                           QuadState_start.foot.HL.pos + Vector3d(0,0,pos_z[2]));
//        state_motion.foot.FR.pos = MovePos(t_temp, interval,
//                                           QuadState_start.foot.FR.pos + Vector3d(0,0,pos_z[1]),
//                                           QuadState_start.foot.FR.pos + Vector3d(0,0,pos_z[2]));
//        state_motion.foot.FL.pos = MovePos(t_temp, interval,
//                                           QuadState_start.foot.FL.pos + Vector3d(0,0,pos_z[1]),
//                                           QuadState_start.foot.FL.pos + Vector3d(0,0,pos_z[2]));

        state_motion.foot.HR.pos = MovePos(t_temp, interval,
                                           QuadState_start.foot.HR.pos + Vector3d(0,0,pos_z[1]+pos_z_ofs) - 0.5*(pos_step+FootStepRotation(pos_rotate,HR)),
                                           QuadState_start.foot.HR.pos + Vector3d(0,0,pos_z[2]) + 0.5*(pos_step+FootStepRotation(pos_rotate,HR)));
        state_motion.foot.HL.pos = MovePos(t_temp, interval,
                                           QuadState_start.foot.HL.pos + Vector3d(0,0,pos_z[1]+pos_z_ofs) - 0.5*(pos_step+FootStepRotation(pos_rotate,HL)),
                                           QuadState_start.foot.HL.pos + Vector3d(0,0,pos_z[2]) + 0.5*(pos_step+FootStepRotation(pos_rotate,HL)));
        state_motion.foot.FR.pos = MovePos(t_temp, interval,
                                           QuadState_start.foot.FR.pos + Vector3d(0,0,pos_z[1]) - 0.5*(pos_step+FootStepRotation(pos_rotate,FR)),
                                           QuadState_start.foot.FR.pos + Vector3d(0,0,pos_z[2]) + 0.5*(pos_step+FootStepRotation(pos_rotate,FR)));
        state_motion.foot.FL.pos = MovePos(t_temp, interval,
                                           QuadState_start.foot.FL.pos + Vector3d(0,0,pos_z[1]) - 0.5*(pos_step+FootStepRotation(pos_rotate,FL)),
                                           QuadState_start.foot.FL.pos + Vector3d(0,0,pos_z[2]) + 0.5*(pos_step+FootStepRotation(pos_rotate,FL)));

    }

    else if(step_num == 5){
    } //------------------------------------PAUSE--------------------------------------------------

    else if(step_num == 6){ // ------------------ motion 06 --------------------- landing
        t_temp = t_sim-timeline[step_num];
        interval = interval_temp[step_num/2];

//        state_motion.foot.HR.pos = MovePos(t_temp, interval,
//                                           QuadState_start.foot.HR.pos + Vector3d(0,0,pos_z[2]),
//                                           QuadState_start.foot.HR.pos + Vector3d(0,0,pos_z[3]));
//        state_motion.foot.HL.pos = MovePos(t_temp, interval,
//                                           QuadState_start.foot.HL.pos + Vector3d(0,0,pos_z[2]),
//                                           QuadState_start.foot.HL.pos + Vector3d(0,0,pos_z[3]));
//        state_motion.foot.FR.pos = MovePos(t_temp, interval,
//                                           QuadState_start.foot.FR.pos + Vector3d(0,0,pos_z[2]),
//                                           QuadState_start.foot.FR.pos + Vector3d(0,0,pos_z[3]));
//        state_motion.foot.FL.pos = MovePos(t_temp, interval,
//                                           QuadState_start.foot.FL.pos + Vector3d(0,0,pos_z[2]),
//                                           QuadState_start.foot.FL.pos + Vector3d(0,0,pos_z[3]));

        state_motion.foot.HR.pos = MovePos(t_temp, interval,
                                           QuadState_start.foot.HR.pos + Vector3d(0,0,pos_z[2]) + 0.5*(pos_step+FootStepRotation(pos_rotate,HR)),
                                           QuadState_start.foot.HR.pos + Vector3d(0,0,pos_z[3]));
        state_motion.foot.HL.pos = MovePos(t_temp, interval,
                                           QuadState_start.foot.HL.pos + Vector3d(0,0,pos_z[2]) + 0.5*(pos_step+FootStepRotation(pos_rotate,HL)),
                                           QuadState_start.foot.HL.pos + Vector3d(0,0,pos_z[3]));
        state_motion.foot.FR.pos = MovePos(t_temp, interval,
                                           QuadState_start.foot.FR.pos + Vector3d(0,0,pos_z[2]) + 0.5*(pos_step+FootStepRotation(pos_rotate,FR)),
                                           QuadState_start.foot.FR.pos + Vector3d(0,0,pos_z[3]));
        state_motion.foot.FL.pos = MovePos(t_temp, interval,
                                           QuadState_start.foot.FL.pos + Vector3d(0,0,pos_z[2]) + 0.5*(pos_step+FootStepRotation(pos_rotate,FL)),
                                           QuadState_start.foot.FL.pos + Vector3d(0,0,pos_z[3]));

    }

    else if(step_num == 7){
    } //------------------------------------PAUSE--------------------------------------------------

    else if(step_num == 8){ // ------------------ motion 08 --------------------- recover
        t_temp = t_sim-timeline[step_num];
        interval = interval_temp[step_num/2];

//        state_motion.foot.HR.pos = MovePos(t_temp, interval,
//                                           QuadState_start.foot.HR.pos + Vector3d(0,0,pos_z[3]),
//                                           QuadState_start.foot.HR.pos + Vector3d(0,0,pos_z[4]));
//        state_motion.foot.HL.pos = MovePos(t_temp, interval,
//                                           QuadState_start.foot.HL.pos + Vector3d(0,0,pos_z[3]),
//                                           QuadState_start.foot.HL.pos + Vector3d(0,0,pos_z[4]));
//        state_motion.foot.FR.pos = MovePos(t_temp, interval,
//                                           QuadState_start.foot.FR.pos + Vector3d(0,0,pos_z[3]),
//                                           QuadState_start.foot.FR.pos + Vector3d(0,0,pos_z[4]));
//        state_motion.foot.FL.pos = MovePos(t_temp, interval,
//                                           QuadState_start.foot.FL.pos + Vector3d(0,0,pos_z[3]),
//                                           QuadState_start.foot.FL.pos + Vector3d(0,0,pos_z[4]));

        state_motion.foot.HR.pos = MovePos(t_temp, interval,
                                           QuadState_start.foot.HR.pos + Vector3d(0,0,pos_z[3]),
                                           QuadState_start.foot.HR.pos + Vector3d(0,0,pos_z[4]));
        state_motion.foot.HL.pos = MovePos(t_temp, interval,
                                           QuadState_start.foot.HL.pos + Vector3d(0,0,pos_z[3]),
                                           QuadState_start.foot.HL.pos + Vector3d(0,0,pos_z[4]));
        state_motion.foot.FR.pos = MovePos(t_temp, interval,
                                           QuadState_start.foot.FR.pos + Vector3d(0,0,pos_z[3]),
                                           QuadState_start.foot.FR.pos + Vector3d(0,0,pos_z[4]));
        state_motion.foot.FL.pos = MovePos(t_temp, interval,
                                           QuadState_start.foot.FL.pos + Vector3d(0,0,pos_z[3]),
                                           QuadState_start.foot.FL.pos + Vector3d(0,0,pos_z[4]));


        if(t_temp >= interval - dt_main) {
            //cout << "check!" << endl;
            if(start_flag == true) {
                cnt_motion = timeline[0]/dt_main - 1;
            }
        }
    }

    else if(step_num == 9){
    } //------------------------------------PAUSE--------------------------------------------------

    else{
        t_temp = t_sim-timeline[10];

        step_num = -1;
        if(stop_flag == true || onestep_flag == true) {
            ready_flag = true;
            start_flag = false;
            stop_flag = false;
            onestep_flag = false;
            mode_state = 0;
            hopping_init_flag = true;
        }
        cnt_motion = -1;
    }

    //--------------------------------------------------------------------------------------------------
    // COM control parameter

    Vector3d com_Kp_f = Vector3d(sharedData->control_variable_double[1] ,sharedData->control_variable_double[3], sharedData->control_variable_double[5]);
    Vector3d com_Kd_f = Vector3d(sharedData->control_variable_double[2] ,sharedData->control_variable_double[4], sharedData->control_variable_double[6]);

    Matrix3d com_Kp_mat_f = diagonalize(com_Kp_f);
    Matrix3d com_Kd_mat_f = diagonalize(com_Kd_f);

    com_control_force = com_Kp_mat_f*-QuadSensor.IMU.angle + com_Kd_mat_f*-QuadSensor.IMU.vel;


    double limit_f[3] = {300, 300, 300};
    for(int i=0; i<3; i++) {
        if(com_control_force[i] > limit_f[i]) com_control_force[i] = limit_f[i];
        else if(com_control_force[i] < -limit_f[i]) com_control_force[i] = -limit_f[i];
    }

    foot_Force_COM[HR] = Vector3d(0, 0, +com_control_force.x() -com_control_force.y());
    foot_Force_COM[HL] = Vector3d(0, 0, -com_control_force.x() -com_control_force.y());
    foot_Force_COM[FR] = Vector3d(0, 0, +com_control_force.x() +com_control_force.y());
    foot_Force_COM[FL] = Vector3d(0, 0, -com_control_force.x() +com_control_force.y());

    // ------------------------- Position reference output ------------------------
    state_out = state_motion;

    // ------------------------ Task space position control --------------------
    for(int i=0; i<3; i++){
        Kp[i] = sharedData->Kp[i];
        Kd[i] = sharedData->Kd[i];
    }
    Kp_mat = diagonalize(Kp);
    Kd_mat = diagonalize(Kd);

    current_limit_ = sharedData->Cmax;

    foot_Force_ref[HR] = Kp_mat*(state_out.foot.HR.pos-QuadState_NOIMU.foot.HR.pos) + Kd_mat*(-QuadState_NOIMU.foot.HR.vel);
    foot_Force_ref[HL] = Kp_mat*(state_out.foot.HL.pos-QuadState_NOIMU.foot.HL.pos) + Kd_mat*(-QuadState_NOIMU.foot.HL.vel);
    foot_Force_ref[FR] = Kp_mat*(state_out.foot.FR.pos-QuadState_NOIMU.foot.FR.pos) + Kd_mat*(-QuadState_NOIMU.foot.FR.vel);
    foot_Force_ref[FL] = Kp_mat*(state_out.foot.FL.pos-QuadState_NOIMU.foot.FL.pos) + Kd_mat*(-QuadState_NOIMU.foot.FL.vel);

    // ------------------------ Gravity compensation (contact based) --------------------

    double mg = -280;
    for(int i=0; i<4; i++) {
        foot_Force_COM[i] /= 4;

        if(step_num < 4 || step_num >= 8) {
            if(step_num == 2) {
                foot_Force_FF[i].z() -= 20;
                if(foot_Force_FF[i].z() < mg) foot_Force_FF[i].z() = mg;
            }
            else {
                foot_Force_FF[i].z() -= 5;
                if(foot_Force_FF[i].z() < mg/4) foot_Force_FF[i].z() = mg/4;
            }
//            foot_Force_FF[i].z() -= 5;
//            if(foot_Force_FF[i].z() < -70) foot_Force_FF[i].z() = -70;
        }
        else
            foot_Force_FF[i].z() = 0;
    }




    // ------------------------ SUM feedforward force --------------------

    QuadStateRef.foot.HR.force = foot_Force_ref[HR] + foot_Force_FF[HR] + foot_Force_COM[HR];
    QuadStateRef.foot.HL.force = foot_Force_ref[HL] + foot_Force_FF[HL] + foot_Force_COM[HL];
    QuadStateRef.foot.FR.force = foot_Force_ref[FR] + foot_Force_FF[FR] + foot_Force_COM[FR];
    QuadStateRef.foot.FL.force = foot_Force_ref[FL] + foot_Force_FF[FL] + foot_Force_COM[FL];

    //printf("%3.2f %3.2f %3.2f\n", foot_Force_ref[FL].x(),foot_Force_ref[FL].y(),foot_Force_ref[FL].z());

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

//    CurRef.HRR = CurRef_temp.HRR;
//    CurRef.HRP = CurRef_temp.HRP;
//    CurRef.HRK = CurRef_temp.HRK - (CurRef_temp.HRP)/11;
//    CurRef.HLR = CurRef_temp.HLR;
//    CurRef.HLP = CurRef_temp.HLP;
//    CurRef.HLK = CurRef_temp.HLK - (CurRef_temp.HLP)/11;
//    CurRef.FRR = CurRef_temp.FRR;
//    CurRef.FRP = CurRef_temp.FRP;
//    CurRef.FRK = CurRef_temp.FRK - (CurRef_temp.FRP)/11;
//    CurRef.FLR = CurRef_temp.FLR;
//    CurRef.FLP = CurRef_temp.FLP;
//    CurRef.FLK = CurRef_temp.FLK - (CurRef_temp.FLP)/11;

    CurRef.HRR = CurRef_temp.HRR + ComputedCurRef.HRR;
    CurRef.HRP = CurRef_temp.HRP + ComputedCurRef.HRP;
    CurRef.HRK = CurRef_temp.HRK + ComputedCurRef.HRK - (CurRef_temp.HRP + ComputedCurRef.HRP)/11;
    CurRef.HLR = CurRef_temp.HLR + ComputedCurRef.HLR;
    CurRef.HLP = CurRef_temp.HLP + ComputedCurRef.HLP;
    CurRef.HLK = CurRef_temp.HLK + ComputedCurRef.HLK - (CurRef_temp.HLP + ComputedCurRef.HLP)/11;
    CurRef.FRR = CurRef_temp.FRR + ComputedCurRef.FRR;
    CurRef.FRP = CurRef_temp.FRP + ComputedCurRef.FRP;
    CurRef.FRK = CurRef_temp.FRK + ComputedCurRef.FRK - (CurRef_temp.FRP + ComputedCurRef.FRP)/11;
    CurRef.FLR = CurRef_temp.FLR + ComputedCurRef.FLR;
    CurRef.FLP = CurRef_temp.FLP + ComputedCurRef.FLP;
    CurRef.FLK = CurRef_temp.FLK + ComputedCurRef.FLK - (CurRef_temp.FLP + ComputedCurRef.FLP)/11;

    for(int i=0; i<4; i++) {
        if(sharedData->task_legcon[i]) {
            for(int j=0; j<3; j++) {
                MY_CONTROL_FB_GAIN[i*3+j] = 10;
                MY_CONTROL_FF_GAIN[i*3+j] = 10;
            }
            MY_CONTROL_FB_GAIN[i*3] = 10;

        }
        else {
            for(int j=0; j<3; j++) {
                MY_CONTROL_FB_GAIN[i*3+j] = 10;
                MY_CONTROL_FF_GAIN[i*3+j] = 0;
            }
        }
    }

    if(sharedData->isFalled == true) {
        task_idx = NoTask;
        sharedData->NEWCOMMAND = true;
        sharedData->COMMAND.USER_COMMAND = MOTION_FALLING;
        JointRef = QuadSensorRef.Encoder.pos;

        hopping_init_flag = true;

        cnt_motion = -1;

        cout << "Motion Falled." << endl;

//        sharedData->plot_play_ON = false;
        isMotionEnabled = false;

        return JointRef;
    }

    if(sharedData->isStopping == true) {
        hopping_init_flag = true;

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

    //printf("step : %d, t_sim : %.3f, force : %.3f\n", step_num, t_sim, foot_Force_FF[HR].z());

    return  JointRef;
}
