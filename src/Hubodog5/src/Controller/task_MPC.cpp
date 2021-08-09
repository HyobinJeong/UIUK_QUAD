#include "hubodog5_general.h"
#include "func_sensor.h"

QuadJoint hubodog5_general::task_MPC()
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
        cout << "MPC Control" << endl;
        TaskName = "MPC";

        for(int i=0; i<3; i++){
            Kp[i] = sharedData->Kp[i];
            Kd[i] = sharedData->Kd[i];
        }
        Kp_mat = diagonalize(Kp);
        Kd_mat = diagonalize(Kd);
        QuadState_old = QuadState;
        QuadSensor_old = QuadSensor;
    }


//    if(flag_stance[0] = true)
//    {
//        TorqueRef[HR] = JT * GRF;
//        CurRef[HR] = Torque2Current()
//    }




    if(isSavingData)
        save_onestep(cnt_motion);


    JointRef = QuadSensor_old.Encoder.pos; //test
    cnt_motion++;
    return JointRef;
}
