#include "hubodog5_general.h"
#include "func_general.h"
//#include "state_estimator.h"

double l2sq[4];

double I_hip_link[4];
double I_knee_link[4];
double I_hip_rotor[4];
double I_knee_rotor[4];

double b_hip_rotor[4];
double b_hip_link[4];
double b_knee_rotor[4];
double b_knee_link[4];

double I_leg[3] = {0, 0.004569337+0.008758, 0.002605831};
double I_motor[3] = {0, 0.000072, 0.000064};
double b_leg[3] = {0, };
double b_motor[3] = {0, };
double g_ratio = 11;
double t_roll[4] = {-0.6, 0.6, -0.6, 0.6};

bool contact_flag_pre[4];
Vector3d vel_temp;
Vector3d vel_temp_ref;
Vector3d vel_temp_l;

void hubodog5_general::state_estimator()
{
    if(cnt_motion < 1)
    {

    }
    //Kinematics.ULEG
    double hp2com2 = 0.088;
    l2sq[HR] = Kinematics.ULEG*Kinematics.ULEG + hp2com2*hp2com2 - 2*Kinematics.ULEG*hp2com2*cos(-QuadSensorRef.Encoder.pos.HRK);
    l2sq[HL] = Kinematics.ULEG*Kinematics.ULEG + hp2com2*hp2com2 - 2*Kinematics.ULEG*hp2com2*cos(-QuadSensorRef.Encoder.pos.HLK);
    l2sq[FR] = Kinematics.ULEG*Kinematics.ULEG + hp2com2*hp2com2 - 2*Kinematics.ULEG*hp2com2*cos(-QuadSensorRef.Encoder.pos.FRK);
    l2sq[FL] = Kinematics.ULEG*Kinematics.ULEG + hp2com2*hp2com2 - 2*Kinematics.ULEG*hp2com2*cos(-QuadSensorRef.Encoder.pos.FLK);

    for(int i=0; i<4; i++) {
        I_knee_rotor[i] = 0.000064;
        I_knee_link[i] = 0.002605831;
        I_hip_rotor[i] = 0.000072;
        I_hip_link[i] = 0.004569337 + 0.001263569 + 0.173*l2sq[i];

        b_knee_rotor[i] = 0;
        b_knee_link[i] = 0;
        b_hip_rotor[i] = 0;
        b_hip_link[i] = 0;
    }

    //cout << l2sq[HR] << ", " << l2sq[HL] << ", " << l2sq[FR] << ", " << l2sq[FL] << endl;

    // leg dynamics
    ComputedCurRef.HRR = t_roll[HR]+t_roll[HR]*sin(QuadSensorRef.Encoder.pos.HRR);
    ComputedCurRef.HLR = t_roll[HL]+t_roll[HL]*sin(QuadSensorRef.Encoder.pos.HLR);
    ComputedCurRef.FRR = t_roll[FR]+t_roll[FR]*sin(QuadSensorRef.Encoder.pos.FRR);
    ComputedCurRef.FLR = t_roll[FL]+t_roll[FL]*sin(QuadSensorRef.Encoder.pos.FLR);

    //cout << "c_roll : " << QuadSensorRef.Encoder.current.HLR << ", q_roll : " << QuadSensorRef.Encoder.pos.HLR << endl;

    ComputedCurRef.HRP = ((g_ratio*g_ratio*I_hip_rotor[HR] + I_hip_link[HR])*QuadSensorRef.Encoder.acc.HRP
                                       + (g_ratio*g_ratio*b_hip_rotor[HR] + b_hip_link[HR])*QuadSensorRef.Encoder.vel.HRP) / torque_const;
    ComputedCurRef.HLP = ((g_ratio*g_ratio*I_hip_rotor[HL] + I_hip_link[HL])*QuadSensorRef.Encoder.acc.HLP
                                       + (g_ratio*g_ratio*b_hip_rotor[HL] + b_hip_link[HL])*QuadSensorRef.Encoder.vel.HLP) / torque_const;
    ComputedCurRef.FRP = ((g_ratio*g_ratio*I_hip_rotor[FR] + I_hip_link[FR])*QuadSensorRef.Encoder.acc.FRP
                                       + (g_ratio*g_ratio*b_hip_rotor[FR] + b_hip_link[FR])*QuadSensorRef.Encoder.vel.FRP) / torque_const;
    ComputedCurRef.FLP = ((g_ratio*g_ratio*I_hip_rotor[FL] + I_hip_link[FL])*QuadSensorRef.Encoder.acc.FLP
                                       + (g_ratio*g_ratio*b_hip_rotor[FL] + b_hip_link[FL])*QuadSensorRef.Encoder.vel.FLP) / torque_const;

    ComputedCurRef.HRK = ((g_ratio*g_ratio*I_knee_rotor[HR] + I_knee_link[HR])*QuadSensorRef.Encoder.acc.HRK
                                       + (g_ratio*g_ratio*b_knee_rotor[HR] + b_knee_link[HR])*QuadSensorRef.Encoder.vel.HRK) / torque_const;
    ComputedCurRef.HLK = ((g_ratio*g_ratio*I_knee_rotor[HL] + I_knee_link[HL])*QuadSensorRef.Encoder.acc.HLK
                                       + (g_ratio*g_ratio*b_knee_rotor[HL] + b_knee_link[HL])*QuadSensorRef.Encoder.vel.HLK) / torque_const;
    ComputedCurRef.FRK = ((g_ratio*g_ratio*I_knee_rotor[FR] + I_knee_link[FR])*QuadSensorRef.Encoder.acc.FRK
                                       + (g_ratio*g_ratio*b_knee_rotor[FR] + b_knee_link[FR])*QuadSensorRef.Encoder.vel.FRK) / torque_const;
    ComputedCurRef.FLK = ((g_ratio*g_ratio*I_knee_rotor[FL] + I_knee_link[FL])*QuadSensorRef.Encoder.acc.FLK
                                       + (g_ratio*g_ratio*b_knee_rotor[FL] + b_knee_link[FL])*QuadSensorRef.Encoder.vel.FLK) / torque_const;

    QuadSensorRef.Encoder.current.HRR = CurRef.HRR;
    QuadSensorRef.Encoder.current.HRP = CurRef.HRP;
    QuadSensorRef.Encoder.current.HRK = CurRef.HRK;
    QuadSensorRef.Encoder.current.HLR = CurRef.HLR;
    QuadSensorRef.Encoder.current.HLP = CurRef.HLP;
    QuadSensorRef.Encoder.current.HLK = CurRef.HLK;
    QuadSensorRef.Encoder.current.FRR = CurRef.FRR;
    QuadSensorRef.Encoder.current.FRP = CurRef.FRP;
    QuadSensorRef.Encoder.current.FRK = CurRef.FRK;
    QuadSensorRef.Encoder.current.FLR = CurRef.FLR;
    QuadSensorRef.Encoder.current.FLP = CurRef.FLP;
    QuadSensorRef.Encoder.current.FLK = CurRef.FLK;

    sharedData->joint_info_ref[36+0]  = CurRef.HRR;
    sharedData->joint_info_ref[36+1]  = CurRef.HRP;
    sharedData->joint_info_ref[36+2]  = CurRef.HRK;
    sharedData->joint_info_ref[36+3]  = CurRef.HLR;
    sharedData->joint_info_ref[36+4]  = CurRef.HLP;
    sharedData->joint_info_ref[36+5]  = CurRef.HLK;
    sharedData->joint_info_ref[36+6]  = CurRef.FRR;
    sharedData->joint_info_ref[36+7]  = CurRef.FRP;
    sharedData->joint_info_ref[36+8]  = CurRef.FRK;
    sharedData->joint_info_ref[36+9]  = CurRef.FLR;
    sharedData->joint_info_ref[36+10] = CurRef.FLP;
    sharedData->joint_info_ref[36+11] = CurRef.FLK;



    QuadStateRef = Kinematics.FK_COM(QuadSensorRef, Vector3d(x0_start,0,z0_start));

    static RobotState state_old;

    for(int i=0; i<3; i++) {
        QuadStateRef.foot.HR.vel[i] = (QuadStateRef.foot.HR.pos[i] - state_old.foot.HR.pos[i])/dt_main;
        QuadStateRef.foot.HR.acc[i] = (QuadStateRef.foot.HR.vel[i] - state_old.foot.HR.vel[i])/dt_main;

        QuadStateRef.foot.HL.vel[i] = (QuadStateRef.foot.HL.pos[i] - state_old.foot.HL.pos[i])/dt_main;
        QuadStateRef.foot.HL.acc[i] = (QuadStateRef.foot.HL.vel[i] - state_old.foot.HL.vel[i])/dt_main;

        QuadStateRef.foot.FR.vel[i] = (QuadStateRef.foot.FR.pos[i] - state_old.foot.FR.pos[i])/dt_main;
        QuadStateRef.foot.FR.acc[i] = (QuadStateRef.foot.FR.vel[i] - state_old.foot.FR.vel[i])/dt_main;

        QuadStateRef.foot.FL.vel[i] = (QuadStateRef.foot.FL.pos[i] - state_old.foot.FL.pos[i])/dt_main;
        QuadStateRef.foot.FL.acc[i] = (QuadStateRef.foot.FL.vel[i] - state_old.foot.FL.vel[i])/dt_main;

        state_old.foot.HR.pos[i] = QuadStateRef.foot.HR.pos[i];
        state_old.foot.HR.vel[i] = QuadStateRef.foot.HR.vel[i];

        state_old.foot.HL.pos[i] = QuadStateRef.foot.HL.pos[i];
        state_old.foot.HL.vel[i] = QuadStateRef.foot.HL.vel[i];

        state_old.foot.FR.pos[i] = QuadStateRef.foot.FR.pos[i];
        state_old.foot.FR.vel[i] = QuadStateRef.foot.FR.vel[i];

        state_old.foot.FL.pos[i] = QuadStateRef.foot.FL.pos[i];
        state_old.foot.FL.vel[i] = QuadStateRef.foot.FL.vel[i];
    }

    //printf("step:%d, time:%.3f, pos x:%.4f, vel x x:%.4f\n", step_num, t_sim, QuadStateRef.foot.HR.pos[0], QuadStateRef.foot.HR.vel[0]);


    sharedData->pf_ref[0+0] =   QuadStateRef.foot.HR.pos[0];
    sharedData->pf_ref[0+1] =   QuadStateRef.foot.HR.pos[1];
    sharedData->pf_ref[0+2] =   QuadStateRef.foot.HR.pos[2];
    sharedData->pf_ref[0+3] =   QuadStateRef.foot.HL.pos[0];
    sharedData->pf_ref[0+4] =   QuadStateRef.foot.HL.pos[1];
    sharedData->pf_ref[0+5] =   QuadStateRef.foot.HL.pos[2];
    sharedData->pf_ref[0+6] =   QuadStateRef.foot.FR.pos[0];
    sharedData->pf_ref[0+7] =   QuadStateRef.foot.FR.pos[1];
    sharedData->pf_ref[0+8] =   QuadStateRef.foot.FR.pos[2];
    sharedData->pf_ref[0+9] =   QuadStateRef.foot.FL.pos[0];
    sharedData->pf_ref[0+10] =  QuadStateRef.foot.FL.pos[1];
    sharedData->pf_ref[0+11] =  QuadStateRef.foot.FL.pos[2];

    sharedData->pf_ref[12+0] =   QuadStateRef.foot.HR.vel[0];
    sharedData->pf_ref[12+1] =   QuadStateRef.foot.HR.vel[1];
    sharedData->pf_ref[12+2] =   QuadStateRef.foot.HR.vel[2];
    sharedData->pf_ref[12+3] =   QuadStateRef.foot.HL.vel[0];
    sharedData->pf_ref[12+4] =   QuadStateRef.foot.HL.vel[1];
    sharedData->pf_ref[12+5] =   QuadStateRef.foot.HL.vel[2];
    sharedData->pf_ref[12+6] =   QuadStateRef.foot.FR.vel[0];
    sharedData->pf_ref[12+7] =   QuadStateRef.foot.FR.vel[1];
    sharedData->pf_ref[12+8] =   QuadStateRef.foot.FR.vel[2];
    sharedData->pf_ref[12+9] =   QuadStateRef.foot.FL.vel[0];
    sharedData->pf_ref[12+10] =  QuadStateRef.foot.FL.vel[1];
    sharedData->pf_ref[12+11] =  QuadStateRef.foot.FL.vel[2];

    sharedData->pf_ref[24+0] =   QuadStateRef.foot.HR.acc[0];
    sharedData->pf_ref[24+1] =   QuadStateRef.foot.HR.acc[1];
    sharedData->pf_ref[24+2] =   QuadStateRef.foot.HR.acc[2];
    sharedData->pf_ref[24+3] =   QuadStateRef.foot.HL.acc[0];
    sharedData->pf_ref[24+4] =   QuadStateRef.foot.HL.acc[1];
    sharedData->pf_ref[24+5] =   QuadStateRef.foot.HL.acc[2];
    sharedData->pf_ref[24+6] =   QuadStateRef.foot.FR.acc[0];
    sharedData->pf_ref[24+7] =   QuadStateRef.foot.FR.acc[1];
    sharedData->pf_ref[24+8] =   QuadStateRef.foot.FR.acc[2];
    sharedData->pf_ref[24+9] =   QuadStateRef.foot.FL.acc[0];
    sharedData->pf_ref[24+10] =  QuadStateRef.foot.FL.acc[1];
    sharedData->pf_ref[24+11] =  QuadStateRef.foot.FL.acc[2];


    sharedData->pf_ref[36+0] =   QuadStateRef.foot.HR.force[0];
    sharedData->pf_ref[36+1] =   QuadStateRef.foot.HR.force[1];
    sharedData->pf_ref[36+2] =   QuadStateRef.foot.HR.force[2];
    sharedData->pf_ref[36+3] =   QuadStateRef.foot.HL.force[0];
    sharedData->pf_ref[36+4] =   QuadStateRef.foot.HL.force[1];
    sharedData->pf_ref[36+5] =   QuadStateRef.foot.HL.force[2];
    sharedData->pf_ref[36+6] =   QuadStateRef.foot.FR.force[0];
    sharedData->pf_ref[36+7] =   QuadStateRef.foot.FR.force[1];
    sharedData->pf_ref[36+8] =   QuadStateRef.foot.FR.force[2];
    sharedData->pf_ref[36+9] =   QuadStateRef.foot.FL.force[0];
    sharedData->pf_ref[36+10] =  QuadStateRef.foot.FL.force[1];
    sharedData->pf_ref[36+11] =  QuadStateRef.foot.FL.force[2];

    sharedData->pcom_ref[0+0] = QuadStateRef.COM.pos[0];
    sharedData->pcom_ref[0+1] = QuadStateRef.COM.pos[1];
    sharedData->pcom_ref[0+2] = QuadStateRef.COM.pos[2];


    QuadState = Kinematics.FK_COM_IMU(QuadSensor, Vector3d(x0_start,0,z0_start));

    //for estimation
    QuadState_NOIMU = Kinematics.FK_COM(QuadSensor, Vector3d(x0_start,0,z0_start));

    QuadStateIMU = Kinematics.FK_COM_IMU(QuadSensor, Vector3d(x0_start,0,z0_start));

    Vector3d COM_pos;
    Vector3d COM_pos_ref;
    Vector3d COM_pos_temp[4];
    int n_contact = 0;

    COM_pos_temp[HR] = -QuadStateIMU.foot.HR.pos;
    COM_pos_temp[HL] = -QuadStateIMU.foot.HL.pos;
    COM_pos_temp[FR] = -QuadStateIMU.foot.FR.pos;
    COM_pos_temp[FL] = -QuadStateIMU.foot.FL.pos;

    if(sharedData->contact_flag[HR] == true) {       
        if(contact_flag_pre[HR]==false){
            Vector3d del_contact = -(QuadStateIMU.foot.HL.pos+QuadStateIMU.foot.FR.pos)/2 + (QuadStateIMU.foot.HR.pos + QuadStateIMU.foot.FL.pos)/2;
            del_contact.z() = 0;
            Contact_point = Contact_point + del_contact;

            Vector3d del_contact_ref = -(QuadStateRef.foot.HL.pos+QuadStateRef.foot.FR.pos)/2 + (QuadStateRef.foot.HR.pos + QuadStateRef.foot.FL.pos)/2;
            del_contact_ref.z() = 0;
            Contact_point_ref = Contact_point_ref + del_contact_ref;
        }
        COM_pos = Contact_point - (QuadStateIMU.foot.HR.pos + QuadStateIMU.foot.FL.pos)/2;
        COM_pos_ref = Contact_point_ref - (QuadStateRef.foot.HR.pos + QuadStateRef.foot.FL.pos)/2;
    }
    else if(sharedData->contact_flag[HL] == true) {
        if(contact_flag_pre[HL]==false){
            Vector3d del_contact = -(QuadStateIMU.foot.HR.pos + QuadStateIMU.foot.FL.pos)/2 + (QuadStateIMU.foot.HL.pos + QuadStateIMU.foot.FR.pos)/2;
            del_contact.z() = 0;
            Contact_point = Contact_point + del_contact;

            Vector3d del_contact_ref = -(QuadStateRef.foot.HR.pos + QuadStateRef.foot.FL.pos)/2 + (QuadStateRef.foot.HL.pos + QuadStateRef.foot.FR.pos)/2;
            del_contact_ref.z() = 0;
            Contact_point_ref = Contact_point_ref + del_contact_ref;
        }
        COM_pos = Contact_point - (QuadStateIMU.foot.HL.pos + QuadStateIMU.foot.FR.pos)/2;
        COM_pos_ref = Contact_point_ref - (QuadStateRef.foot.HL.pos + QuadStateRef.foot.FR.pos)/2;
    }
    else if(sharedData->contact_flag[HR] == false && sharedData->contact_flag[HL] == false){
        Contact_point += ComLast_vel*dt_main;
        COM_pos += ComLast_vel*dt_main;
    }

    if(sharedData->contact_flag[HR] == false){
        if(contact_flag_pre[HR] == true){
            ComLast_vel = Vector3dZero;//ComStateRef.vel;
        }
    }
    if(sharedData->contact_flag[HL] == false){
        if(contact_flag_pre[HL] == true){
            ComLast_vel = Vector3dZero;//ComStateRef.vel;
        }
    }




    for(int i=0; i<4; i++) {
        contact_flag_pre[i] = sharedData->contact_flag[i];
    }




//    for(int i=0; i<2; i++) {

//        if(sharedData->contact_flag[i]==true && contact_flag_pre[i]==false) {

//        }

//        if(sharedData->contact_flag[i] == true) {
//            for(int j=0; j<3; j++) {
//                COM_pos[j] += COM_pos_temp[i][j];
//            }
//        }
//        n_contact += (int)sharedData->contact_flag[i];
//    }

    for(int i=0; i<3; i++) {
        //COM_pos[i] /= (double)n_contact;
        //ComState.pos[i] = COM_pos[i];

        // need to test and modify
        if(COM_pos[i] < 1000.0 && COM_pos[i] > -1000.0)
            //ComState.pos[i] = COM_pos[i];
            ComState.pos[i] = Lowpass_filter(COM_pos[i],ComState.pos[i],2.0 ,dt_main);

        if(COM_pos_ref[i] < 1000.0 && COM_pos_ref[i] > -1000.0)
//            ComStateRef.pos[i] = COM_pos_ref[i];
            ComStateRef.pos[i] = Lowpass_filter(COM_pos_ref[i],ComStateRef.pos[i],2.0,dt_main);

    }


    for(int i=0; i<3; i++) {
        double vel_limit = 0.05;
        vel_temp[i] = (ComState.pos[i] - ComState_old.pos[i])/dt_main;
        vel_temp_ref[i] = (ComStateRef.pos[i] - ComStateRef_old.pos[i])/dt_main;

//        if(vel_temp[i] < vel_limit && vel_temp[i] > -vel_limit) {
//            vel_temp[i] = 0;
//        }

//        if(vel_temp[i] > vel_limit) {
//            vel_temp_l[i] = vel_temp[i] - vel_limit;
//        }
//        else if(vel_temp[i] < -vel_limit) {
//            vel_temp_l[i] = vel_temp[i] + vel_limit;
//        }
//        else {
//            vel_temp_l[i] = 0;
//        }

        ComState.vel[i] = Lowpass_filter(vel_temp[i],ComState.vel[i],10.0,dt_main);
        ComStateRef.vel[i] = Lowpass_filter(vel_temp_ref[i],ComStateRef.vel[i],100.0,dt_main);
        //ComStateRef.vel[i] = vel_temp_ref[i];

        ComState.acc[i] = (ComState.vel[i] - ComState_old.vel[i])/dt_main;
        ComStateRef.acc[i] = (ComStateRef.vel[i] - ComStateRef_old.vel[i])/dt_main;

    }

    ComState_old.pos = ComState.pos;
    ComStateRef_old.pos = ComStateRef.pos;

    ComState_old.vel = ComState.vel;
    ComStateRef_old.vel = ComStateRef.vel;


    //cout << "n_contact : " << n_contact << ", x:" << ComState.pos.x() << ", xin:" <<COM_pos.x() << endl;


    sharedData->pf_real[0+0] =   QuadState.foot.HR.pos[0];
    sharedData->pf_real[0+1] =   QuadState.foot.HR.pos[1];
    sharedData->pf_real[0+2] =   QuadState.foot.HR.pos[2];
    sharedData->pf_real[0+3] =   QuadState.foot.HL.pos[0];
    sharedData->pf_real[0+4] =   QuadState.foot.HL.pos[1];
    sharedData->pf_real[0+5] =   QuadState.foot.HL.pos[2];
    sharedData->pf_real[0+6] =   QuadState.foot.FR.pos[0];
    sharedData->pf_real[0+7] =   QuadState.foot.FR.pos[1];
    sharedData->pf_real[0+8] =   QuadState.foot.FR.pos[2];
    sharedData->pf_real[0+9] =   QuadState.foot.FL.pos[0];
    sharedData->pf_real[0+10] =  QuadState.foot.FL.pos[1];
    sharedData->pf_real[0+11] =  QuadState.foot.FL.pos[2];

    sharedData->pcom_real[0+0] = ComState.pos[0];
    sharedData->pcom_real[0+1] = ComState.pos[1];
    sharedData->pcom_real[0+2] = ComState.pos[2];
    sharedData->pcom_real[3+0] = ComState.vel[0];
    sharedData->pcom_real[3+1] = ComState.vel[1];
    sharedData->pcom_real[3+2] = ComState.vel[2];

    Matrix3d jacobian;
    Vector3d JointVel;
    Vector3d joint_trq;
    Vector3d FootVel, force;
    Vector3d FootVel_global, force_global;

    QuadJoint current_temp;

    current_temp.HRR = QuadSensor.Encoder.current.HRR;
    current_temp.HRP = QuadSensor.Encoder.current.HRP;
    current_temp.HRK = QuadSensor.Encoder.current.HRK;
    current_temp.HLR = QuadSensor.Encoder.current.HLR;
    current_temp.HLP = QuadSensor.Encoder.current.HLP;
    current_temp.HLK = QuadSensor.Encoder.current.HLK;
    current_temp.FRR = QuadSensor.Encoder.current.FRR;
    current_temp.FRP = QuadSensor.Encoder.current.FRP;
    current_temp.FRK = QuadSensor.Encoder.current.FRK;
    current_temp.FLR = QuadSensor.Encoder.current.FLR;
    current_temp.FLP = QuadSensor.Encoder.current.FLP;
    current_temp.FLK = QuadSensor.Encoder.current.FLK;

//    current_temp.HRR = QuadSensor.Encoder.current.HRR - QuadSensorRef.Encoder.current.HRR;
//    current_temp.HRP = QuadSensor.Encoder.current.HRP - QuadSensorRef.Encoder.current.HRP;
//    current_temp.HRK = QuadSensor.Encoder.current.HRK - QuadSensorRef.Encoder.current.HRK;
//    current_temp.HLR = QuadSensor.Encoder.current.HLR - QuadSensorRef.Encoder.current.HLR;
//    current_temp.HLP = QuadSensor.Encoder.current.HLP - QuadSensorRef.Encoder.current.HLP;
//    current_temp.HLK = QuadSensor.Encoder.current.HLK - QuadSensorRef.Encoder.current.HLK;
//    current_temp.FRR = QuadSensor.Encoder.current.FRR - QuadSensorRef.Encoder.current.FRR;
//    current_temp.FRP = QuadSensor.Encoder.current.FRP - QuadSensorRef.Encoder.current.FRP;
//    current_temp.FRK = QuadSensor.Encoder.current.FRK - QuadSensorRef.Encoder.current.FRK;
//    current_temp.FLR = QuadSensor.Encoder.current.FLR - QuadSensorRef.Encoder.current.FLR;
//    current_temp.FLP = QuadSensor.Encoder.current.FLP - QuadSensorRef.Encoder.current.FLP;
//    current_temp.FLK = QuadSensor.Encoder.current.FLK - QuadSensorRef.Encoder.current.FLK;


    Matrix3d skew_pel_v;
//    skew_pel_v << 0,0, sharedData->IMU[1].PitchVel*D2Rf,
//                  0,0,-sharedData->IMU[1].RollVel*D2Rf,
//                 -sharedData->IMU[1].PitchVel*D2Rf, sharedData->IMU[1].RollVel*D2Rf,0;

    skew_pel_v << 0, -sharedData->IMU[1].YawVel*D2Rf, sharedData->IMU[1].PitchVel*D2Rf,
                  sharedData->IMU[1].YawVel*D2Rf, 0, -sharedData->IMU[1].RollVel*D2Rf,
                 -sharedData->IMU[1].PitchVel*D2Rf, sharedData->IMU[1].RollVel*D2Rf,0;

//    skew_pel_v << 0, 0, 0,
//                  0, 0, 0,
//                 0, 0,0;

    //RampQuatRef = R2Quat(Euler2R(RampEulerRef));
    RampMatRef = Euler2R(RampEulerRef);

    legJacobian[HR] = Kinematics.getOneLegJacobian(Vector3d(QuadSensor.Encoder.pos.HRR, QuadSensor.Encoder.pos.HRP, QuadSensor.Encoder.pos.HRK), -1);
    legJacobian_global[HR] = Kinematics.getOneLegJacobian_Global(QuadSensor, Vector3d(QuadSensor.Encoder.pos.HRR, QuadSensor.Encoder.pos.HRP, QuadSensor.Encoder.pos.HRK), -1);
    legJacobian_angleRef[HR] = Kinematics.getOneLegJacobian_angleRef(RampMatRef, Vector3d(QuadSensor.Encoder.pos.HRR, QuadSensor.Encoder.pos.HRP, QuadSensor.Encoder.pos.HRK), -1);
    JointVel = Vector3d(QuadSensor.Encoder.vel.HRR, QuadSensor.Encoder.vel.HRP, QuadSensor.Encoder.vel.HRK);
    FootVel = legJacobian[HR]*JointVel;
    FootVel_global = legJacobian_global[HR]*JointVel + skew_pel_v*QuadStateIMU.foot.HR.pos;
    QuadState_NOIMU.foot.HR.vel = FootVel;
    QuadState.foot.HR.vel = FootVel_global;
    joint_trq = Vector3d(current_temp.HRR, current_temp.HRP, current_temp.HRK) * torque_const;
    force = legJacobian[HR].transpose().inverse() * joint_trq;
    QuadState.foot.HR.force = force;

    legJacobian[HL] = Kinematics.getOneLegJacobian(Vector3d(QuadSensor.Encoder.pos.HLR, QuadSensor.Encoder.pos.HLP, QuadSensor.Encoder.pos.HLK), 1);
    legJacobian_global[HL] = Kinematics.getOneLegJacobian_Global(QuadSensor, Vector3d(QuadSensor.Encoder.pos.HLR, QuadSensor.Encoder.pos.HLP, QuadSensor.Encoder.pos.HLK), 1);
    legJacobian_angleRef[HL] = Kinematics.getOneLegJacobian_angleRef(RampMatRef, Vector3d(QuadSensor.Encoder.pos.HLR, QuadSensor.Encoder.pos.HLP, QuadSensor.Encoder.pos.HLK), 1);
    JointVel = Vector3d(QuadSensor.Encoder.vel.HLR, QuadSensor.Encoder.vel.HLP, QuadSensor.Encoder.vel.HLK);
    FootVel = legJacobian[HL]*JointVel;
    FootVel_global = legJacobian_global[HL]*JointVel + skew_pel_v*QuadStateIMU.foot.HL.pos;
    QuadState_NOIMU.foot.HL.vel = FootVel;
    QuadState.foot.HL.vel = FootVel_global;
    joint_trq = Vector3d(current_temp.HLR, current_temp.HLP, current_temp.HLK) * torque_const;
    force = legJacobian[HL].transpose().inverse() * joint_trq;
    QuadState.foot.HL.force = force;

    legJacobian[FR] = Kinematics.getOneLegJacobian(Vector3d(QuadSensor.Encoder.pos.FRR, QuadSensor.Encoder.pos.FRP, QuadSensor.Encoder.pos.FRK), -1);
    legJacobian_global[FR] = Kinematics.getOneLegJacobian_Global(QuadSensor, Vector3d(QuadSensor.Encoder.pos.FRR, QuadSensor.Encoder.pos.FRP, QuadSensor.Encoder.pos.FRK), -1);
    legJacobian_angleRef[FR] = Kinematics.getOneLegJacobian_angleRef(RampMatRef, Vector3d(QuadSensor.Encoder.pos.FRR, QuadSensor.Encoder.pos.FRP, QuadSensor.Encoder.pos.FRK), -1);
    JointVel = Vector3d(QuadSensor.Encoder.vel.FRR, QuadSensor.Encoder.vel.FRP, QuadSensor.Encoder.vel.FRK);
    FootVel = legJacobian[FR]*JointVel;
    FootVel_global = legJacobian_global[FR]*JointVel + skew_pel_v*QuadStateIMU.foot.FR.pos;
    QuadState_NOIMU.foot.FR.vel = FootVel;
    QuadState.foot.FR.vel = FootVel_global;
    joint_trq = Vector3d(current_temp.FRR, current_temp.FRP, current_temp.FRK) * torque_const;
    force = legJacobian[FR].transpose().inverse() * joint_trq;
    QuadState.foot.FR.force = force;

    legJacobian[FL] = Kinematics.getOneLegJacobian(Vector3d(QuadSensor.Encoder.pos.FLR, QuadSensor.Encoder.pos.FLP, QuadSensor.Encoder.pos.FLK), 1);
    legJacobian_global[FL] = Kinematics.getOneLegJacobian_Global(QuadSensor, Vector3d(QuadSensor.Encoder.pos.FLR, QuadSensor.Encoder.pos.FLP, QuadSensor.Encoder.pos.FLK), 1);
    legJacobian_angleRef[FL] = Kinematics.getOneLegJacobian_angleRef(RampMatRef, Vector3d(QuadSensor.Encoder.pos.FLR, QuadSensor.Encoder.pos.FLP, QuadSensor.Encoder.pos.FLK), 1);
    JointVel = Vector3d(QuadSensor.Encoder.vel.FLR, QuadSensor.Encoder.vel.FLP, QuadSensor.Encoder.vel.FLK);
    FootVel = legJacobian[FL]*JointVel;
    FootVel_global = legJacobian_global[FL]*JointVel + skew_pel_v*QuadStateIMU.foot.FL.pos;
    QuadState_NOIMU.foot.FL.vel = FootVel;
    QuadState.foot.FL.vel = FootVel_global;
    joint_trq = Vector3d(current_temp.FLR, current_temp.FLP, current_temp.FLK) * torque_const;
    force = legJacobian[FL].transpose().inverse() * joint_trq;
    QuadState.foot.FL.force = force;

    sharedData->pf_real[12+0] =   QuadState.foot.HR.vel[0];
    sharedData->pf_real[12+1] =   QuadState.foot.HR.vel[1];
    sharedData->pf_real[12+2] =   QuadState.foot.HR.vel[2];
    sharedData->pf_real[12+3] =   QuadState.foot.HL.vel[0];
    sharedData->pf_real[12+4] =   QuadState.foot.HL.vel[1];
    sharedData->pf_real[12+5] =   QuadState.foot.HL.vel[2];
    sharedData->pf_real[12+6] =   QuadState.foot.FR.vel[0];
    sharedData->pf_real[12+7] =   QuadState.foot.FR.vel[1];
    sharedData->pf_real[12+8] =   QuadState.foot.FR.vel[2];
    sharedData->pf_real[12+9] =   QuadState.foot.FL.vel[0];
    sharedData->pf_real[12+10] =  QuadState.foot.FL.vel[1];
    sharedData->pf_real[12+11] =  QuadState.foot.FL.vel[2];

    sharedData->pf_real[36+0] =   QuadState.foot.HR.force[0];
    sharedData->pf_real[36+1] =   QuadState.foot.HR.force[1];
    sharedData->pf_real[36+2] =   QuadState.foot.HR.force[2];
    sharedData->pf_real[36+3] =   QuadState.foot.HL.force[0];
    sharedData->pf_real[36+4] =   QuadState.foot.HL.force[1];
    sharedData->pf_real[36+5] =   QuadState.foot.HL.force[2];
    sharedData->pf_real[36+6] =   QuadState.foot.FR.force[0];
    sharedData->pf_real[36+7] =   QuadState.foot.FR.force[1];
    sharedData->pf_real[36+8] =   QuadState.foot.FR.force[2];
    sharedData->pf_real[36+9] =   QuadState.foot.FL.force[0];
    sharedData->pf_real[36+10] =  QuadState.foot.FL.force[1];
    sharedData->pf_real[36+11] =  QuadState.foot.FL.force[2];

    //printf("%.2f, %.2f\n", QuadState.foot.FL.force.z(), tempz);

    //cout << "x force : " << QuadState.foot.FL.force[0] << ", y force : " << QuadState.foot.FL.force[1] << ", z force : " << QuadState.foot.FL.force[2] << endl;

    t_sim = cnt_motion * dt_main;
}
