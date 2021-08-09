#include "hubodog5_general.h"
#include "RBSharedMemory.h"

//double control_yaw_vel;
//double control_yaw_pos;
//double control_yaw_pos_hpf;
//double control_yaw_ref;
//double control_yaw_err;
//double control_yaw_err_old;
//double control_yaw_err_dif;

QuadJoint hubodog5_general::task_comcontrol_test()
{
//    QuadJoint JointRef;
//    RobotState state_temp;

//    double hpf_frq = sharedData->comcon_variable_double[0];
//    double yaw_Kp = sharedData->comcon_variable_double[1];
//    double yaw_Kd = sharedData->comcon_variable_double[2];


//    if(isMotionStopping)
//    {
//        ALLPCON();
//        isPosAdjust = true;
//        JointRef = QuadSensor.Encoder.pos;
//        return JointRef;
//    }

//    if(cnt_motion < 1)
//    {
//        cout << "Com Control Test" << endl;
//        TaskName = "Com Control";

//        for(int i=0; i<12; i++){
//            sharedData->pf_ref_init[i] = sharedData->pf_real[i];
//        }

//        QuadState_old = QuadState;
//        QuadSensor_old = QuadSensor;
//    }

//    control_yaw_err_old = control_yaw_err;
//    control_yaw_err = control_yaw_ref - QuadSensor.IMU.vel.z();
//    control_yaw_err_dif = (control_yaw_err - control_yaw_err_old)*500;

//    control_yaw_vel = control_yaw_err*yaw_Kp + control_yaw_err_dif*yaw_Kd;
//    control_yaw_pos += control_yaw_vel*0.002;

//    control_yaw_pos_hpf = Highpass_filter(control_yaw_vel*0.002,0,control_yaw_pos_hpf,hpf_frq,0.002);

//    sharedData->comcon_variable_plot[0] = QuadSensor.IMU.angle.z();
//    sharedData->comcon_variable_plot[1] = QuadSensor.IMU.vel.z();
//    sharedData->comcon_variable_plot[2] = control_yaw_vel;
//    sharedData->comcon_variable_plot[3] = control_yaw_pos;
//    sharedData->comcon_variable_plot[4] = control_yaw_pos_hpf;
//    sharedData->comcon_variable_plot[5] = control_yaw_err_dif;


//    //cout << ",  err : " << control_yaw_err << ", err_dif : " << control_yaw_err_dif << endl;
//    state_temp = QuadState_start;
//    state_temp.COM.pos = Vector3d(x0_start,0,z0_start);
//    state_temp.rotate = Euler2R(Vector3d(0,0,-control_yaw_pos_hpf));
//    //state_temp.rotate = Euler2R(Vector3d(0,0,0));

//    JointRef = Kinematics.IK_FullBody(state_temp);
//    cnt_motion++;
//    return  JointRef;
}
