#include "hubodog5_general.h"
#include "RBSharedMemory.h"

QuadJoint hubodog5_general::task_demo()
{
    QuadJoint JointRef;
    if(isMotionStopping)
    {
        ALLPCON();
        isPosAdjust = true;
        JointRef = QuadSensor.Encoder.pos;
        return JointRef;
    }

    if(cnt_motion < 1)
    {
        cout << "Demo Motion Test" << endl;
        TaskName = "Demo";
        ALLPCON();
    }

    double interval = sharedData->demo_variable[0];
    double interval_pause = 0.5;
    int num_motion = 6;
    double timeline[num_motion*2+2];
    timeline[0] = 0.0;
    for(int i=0; i<num_motion; i++)
    {
        timeline[i*2+1] = timeline[i*2] + interval;
        timeline[i*2+2] = timeline[i*2+1] + interval_pause;
    }
    RobotState state_temp;
    double bias, t_temp;

    if(t_sim<timeline[1]){ // ------------------ motion 01 ---------------------
        t_temp = t_sim-timeline[0];
        bias = sharedData->demo_variable[3]*sin(2*PIf*t_temp/interval);
        state_temp = QuadState_start;
        state_temp.COM.pos = Vector3d(0,0,z0_start+bias);
        JointRef = Kinematics.IK_FullBody(state_temp);

    }
    else if(t_sim>=timeline[1] && t_sim<timeline[2]){
        JointRef = QuadSensor_start.Encoder.pos;
    }
    else if(t_sim>=timeline[2] && t_sim<timeline[3]){ // ------------------ motion 02 ---------------------
        t_temp = t_sim-timeline[2];
        bias = sharedData->demo_variable[1]*sin(2*PIf*t_temp/interval);
        state_temp = QuadState_start;
        state_temp.COM.pos = Vector3d(bias,0,z0_start);
        JointRef = Kinematics.IK_FullBody(state_temp);
    }
    else if(t_sim>=timeline[3] && t_sim<timeline[4]){
        JointRef = QuadSensor_start.Encoder.pos;
    }
    else if(t_sim>=timeline[4] && t_sim<timeline[5]){ // ------------------ motion 03 ---------------------
        t_temp = t_sim-timeline[4];
        bias = sharedData->demo_variable[2]*sin(2*PIf*t_temp/interval);
        state_temp = QuadState_start;
        state_temp.COM.pos = Vector3d(0,bias,z0_start);
        JointRef = Kinematics.IK_FullBody(state_temp);
    }
    else if(t_sim>=timeline[5] && t_sim<timeline[6]){
        JointRef = QuadSensor_start.Encoder.pos;
    }

    else if(t_sim>=timeline[6] && t_sim<timeline[7]){ // ------------------ motion 04 ---------------------
        t_temp = t_sim-timeline[6];
        bias = D2Rf*sharedData->demo_variable[5]*sin(2*PIf*t_temp/interval);
        state_temp = QuadState_start;
        state_temp.COM.pos = Vector3d(0,0,z0_start);
        state_temp.rotate = Euler2R(Vector3d(0,bias,0));
        JointRef = Kinematics.IK_FullBody(state_temp);
    }
    else if(t_sim>=timeline[7] && t_sim<timeline[8]){
        JointRef = QuadSensor_start.Encoder.pos;
    }

    else if(t_sim>=timeline[8] && t_sim<timeline[9]){ // ------------------ motion 05 ---------------------
        t_temp = t_sim-timeline[8];
        bias = D2Rf*sharedData->demo_variable[4]*sin(2*PIf*t_temp/interval);
        state_temp = QuadState_start;
        state_temp.COM.pos = Vector3d(0,0,z0_start);
        state_temp.rotate = Euler2R(Vector3d(bias,0,0));
        JointRef = Kinematics.IK_FullBody(state_temp);
    }
    else if(t_sim>=timeline[9] && t_sim<timeline[10]){
        JointRef = QuadSensor_start.Encoder.pos;
    }

    else if(t_sim>=timeline[10] && t_sim<timeline[11]){ // ------------------ motion 06 ---------------------
        t_temp = t_sim-timeline[10];
        bias = D2Rf*sharedData->demo_variable[6]*sin(2*PIf*t_temp/interval);
        state_temp = QuadState_start;
        state_temp.COM.pos = Vector3d(0,0,z0_start);
        state_temp.rotate = Euler2R(Vector3d(0,0,bias));
        JointRef = Kinematics.IK_FullBody(state_temp);
    }
    else if(t_sim>=timeline[11] && t_sim<timeline[12]){
        JointRef = QuadSensor_start.Encoder.pos;
    }
    else{
        cout << "Demo Motion Finished." << endl;
        motion_stop();
        return  JointRef = QuadSensor_start.Encoder.pos;
    }



    if(sharedData->COMMAND.USER_COMMAND != MOTION_STOP)
        cnt_motion++;
    return  JointRef;
}
