#include "hubodog5_general.h"

QuadJoint hubodog5_general::task_dance01()
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
        cout << "Dance 01 Test" << endl;
        TaskName = "Dance01";
        ALLPCON();
    }

    double interval = 0.3;
    double interval_pause = 0.3;
    int num_motion = 2;
    double timeline[num_motion*2+2];
    timeline[0] = 0.0;
    double step_x = 0.05;
    double step_z = 0.05;
    for(int i=0; i<num_motion; i++)
    {
        timeline[i*2+1] = timeline[i*2] + interval;
        timeline[i*2+2] = timeline[i*2+1] + interval_pause;
    }
    RobotState state_temp;
    double t_temp;
    Vector3d pf_temp;

    if(t_sim<timeline[1]){ // ------------------ motion 01 ---------------------
        t_temp = t_sim-timeline[0];
        state_temp = QuadState_start;
        state_temp.COM.pos = Vector3d(x0_start,0,z0_start);

        pf_temp = bezier_calc(t_temp, interval, Vector3d(state_temp.foot.HR.pos[0],  0,  0), Vector3d(state_temp.foot.HR.pos[0]-step_x, 0, 0));
        state_temp.foot.HR.pos[0] = pf_temp[0];

        pf_temp = bezier_calc(t_temp, interval, Vector3d(state_temp.foot.FL.pos[0],  0,  0), Vector3d(state_temp.foot.FL.pos[0]-step_x, 0, 0));
        state_temp.foot.FL.pos[0] = pf_temp[0];

        pf_temp = bezier_calc(t_temp, interval, Vector3d(state_temp.foot.HL.pos[0],  0,  0), Vector3d(state_temp.foot.HL.pos[0]+step_x, 0, 0));
        state_temp.foot.HL.pos[0] = pf_temp[0];
        pf_temp = bezier_calc(t_temp, interval, Vector3d(0,  0,  0), Vector3d(step_z, 0, 0));
        state_temp.foot.HL.pos[2] = pf_temp[0];

        pf_temp = bezier_calc(t_temp, interval, Vector3d(state_temp.foot.FR.pos[0],  0,  0), Vector3d(state_temp.foot.FR.pos[0]+step_x, 0, 0));
        state_temp.foot.FR.pos[0] = pf_temp[0];
        pf_temp = bezier_calc(t_temp, interval, Vector3d(0,  0,  0), Vector3d(step_z, 0, 0));
        state_temp.foot.FR.pos[2] = pf_temp[0];

        JointRef = Kinematics.IK_FullBody(state_temp);
    }
    else if(t_sim>=timeline[1] && t_sim<timeline[2]){
        t_temp = t_sim-timeline[1];
        state_temp = QuadState_start;
        state_temp.COM.pos = Vector3d(x0_start,0,z0_start);

        pf_temp = bezier_calc(t_temp, interval, Vector3d(state_temp.foot.HR.pos[0]-step_x,  0,  0), Vector3d(state_temp.foot.HR.pos[0]-2*step_x, 0, 0));
        state_temp.foot.HR.pos[0] = pf_temp[0];

        pf_temp = bezier_calc(t_temp, interval, Vector3d(state_temp.foot.FL.pos[0]-step_x,  0,  0), Vector3d(state_temp.foot.FL.pos[0]-2*step_x, 0, 0));
        state_temp.foot.FL.pos[0] = pf_temp[0];

        pf_temp = bezier_calc(t_temp, interval, Vector3d(state_temp.foot.HL.pos[0]+step_x,  0,  0), Vector3d(state_temp.foot.HL.pos[0]+2*step_x, 0, 0));
        state_temp.foot.HL.pos[0] = pf_temp[0];
        pf_temp = bezier_calc(t_temp, interval, Vector3d(step_z,  0,  0), Vector3d(0, 0, 0));
        state_temp.foot.HL.pos[2] = pf_temp[0];

        pf_temp = bezier_calc(t_temp, interval, Vector3d(state_temp.foot.FR.pos[0]+step_x,  0,  0), Vector3d(state_temp.foot.FR.pos[0]+2*step_x, 0, 0));
        state_temp.foot.FR.pos[0] = pf_temp[0];
        pf_temp = bezier_calc(t_temp, interval, Vector3d(step_z,  0,  0), Vector3d(0, 0, 0));
        state_temp.foot.FR.pos[2] = pf_temp[0];

        JointRef = Kinematics.IK_FullBody(state_temp);
    }
    else if(t_sim>=timeline[2] && t_sim<timeline[3]){ // ------------------ motion 02 ---------------------
        t_temp = t_sim-timeline[2];
        state_temp = QuadState_start;
        state_temp.COM.pos = Vector3d(x0_start,0,z0_start);

        pf_temp = bezier_calc(t_temp, interval, Vector3d(state_temp.foot.HR.pos[0]-2*step_x,  0,  0), Vector3d(state_temp.foot.HR.pos[0]-step_x, 0, 0));
        state_temp.foot.HR.pos[0] = pf_temp[0];
        pf_temp = bezier_calc(t_temp, interval, Vector3d(0,  0,  0), Vector3d(step_z, 0, 0));
        state_temp.foot.HR.pos[2] = pf_temp[0];

        pf_temp = bezier_calc(t_temp, interval, Vector3d(state_temp.foot.FL.pos[0]-2*step_x,  0,  0), Vector3d(state_temp.foot.FL.pos[0]-step_x, 0, 0));
        state_temp.foot.FL.pos[0] = pf_temp[0];
        pf_temp = bezier_calc(t_temp, interval, Vector3d(0,  0,  0), Vector3d(step_z, 0, 0));
        state_temp.foot.FL.pos[2] = pf_temp[0];

        pf_temp = bezier_calc(t_temp, interval, Vector3d(state_temp.foot.HL.pos[0]+2*step_x,  0,  0), Vector3d(state_temp.foot.HL.pos[0]+step_x, 0, 0));
        state_temp.foot.HL.pos[0] = pf_temp[0];

        pf_temp = bezier_calc(t_temp, interval, Vector3d(state_temp.foot.FR.pos[0]+2*step_x,  0,  0), Vector3d(state_temp.foot.FR.pos[0]+step_x, 0, 0));
        state_temp.foot.FR.pos[0] = pf_temp[0];

        JointRef = Kinematics.IK_FullBody(state_temp);
    }
    else if(t_sim>=timeline[3] && t_sim<timeline[4]){
        t_temp = t_sim-timeline[3];
        state_temp = QuadState_start;
        state_temp.COM.pos = Vector3d(x0_start,0,z0_start);

        pf_temp = bezier_calc(t_temp, interval, Vector3d(state_temp.foot.HR.pos[0]-step_x,  0,  0), Vector3d(state_temp.foot.HR.pos[0], 0, 0));
        state_temp.foot.HR.pos[0] = pf_temp[0];
        pf_temp = bezier_calc(t_temp, interval, Vector3d(step_z,  0,  0), Vector3d(0, 0, 0));
        state_temp.foot.HR.pos[2] = pf_temp[0];

        pf_temp = bezier_calc(t_temp, interval, Vector3d(state_temp.foot.FL.pos[0]-step_x,  0,  0), Vector3d(state_temp.foot.FL.pos[0], 0, 0));
        state_temp.foot.FL.pos[0] = pf_temp[0];
        pf_temp = bezier_calc(t_temp, interval, Vector3d(step_z,  0,  0), Vector3d(0, 0, 0));
        state_temp.foot.FL.pos[2] = pf_temp[0];

        pf_temp = bezier_calc(t_temp, interval, Vector3d(state_temp.foot.HL.pos[0]+step_x,  0,  0), Vector3d(state_temp.foot.HL.pos[0], 0, 0));
        state_temp.foot.HL.pos[0] = pf_temp[0];

        pf_temp = bezier_calc(t_temp, interval, Vector3d(state_temp.foot.FR.pos[0]+step_x,  0,  0), Vector3d(state_temp.foot.FR.pos[0], 0, 0));
        state_temp.foot.FR.pos[0] = pf_temp[0];

        JointRef = Kinematics.IK_FullBody(state_temp);
    }
    else{
        cnt_motion = 0;
        JointRef = QuadSensor_start.Encoder.pos;

//        cout << "Dance 01 Finished." << endl;
//        motion_stop();
//        return  JointRef = QuadSensor_start.Encoder.pos;
    }


    if(sharedData->COMMAND.USER_COMMAND != MOTION_STOP)
        cnt_motion++;
    return  JointRef;
}
