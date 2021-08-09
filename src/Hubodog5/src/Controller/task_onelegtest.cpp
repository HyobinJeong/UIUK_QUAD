#include "hubodog5_general.h"
#include "func_sensor.h"

QuadJoint hubodog5_general::task_onelegtest()
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
        cout << "OneLeg Test" << endl;
        TaskName = "OneLegTest";
        for(int i=0; i<3; i++){
            Kp[i] = sharedData->Kp[i];
            Kd[i] = sharedData->Kd[i];
        }
        Kp_mat = diagonalize(Kp);
        Kd_mat = diagonalize(Kd);

        for(int i=0; i<12; i++){
            sharedData->pf_ref_init[i] = sharedData->pf_real[i];
        }

        QuadState_old = QuadState;
        QuadSensor_old = QuadSensor;
    }

    if(sharedData->OneLegTest_ON)
    {
        sharedData->OneLegTest_ON = false;
        for(int i=0; i<3; i++){
            Kp[i] = sharedData->Kp[i];
            Kd[i] = sharedData->Kd[i];
        }
        Kp_mat = diagonalize(Kp);
        Kd_mat = diagonalize(Kd);

        for(int i=0; i<4; i++)
        {
            if(sharedData->idx_legcon[i])
            {
//                if(!isSimulation)
                    for(int j=0; j<3; j++) {Ccon[i*3+j]=true;}
            }
            else{
                for(int j=0; j<3; j++) {Pcon[i*3+j]=true;}
            }
        }

        current_limit_ = sharedData->Cmax;
    }

    if(sharedData->OneLegTest_OFF)
    {
        sharedData->OneLegTest_OFF = false;
        for(int i=0; i<NO_OF_JOINTS; i++) {Pcon[i]=true;}
        QuadSensor_old = QuadSensor;
    }

    Vector3d JointPos, JointVel, FootPos, FootVel, FootPosRef, FootPosRef_local, JointPos_IK;
    Vector3d Torque_HR, Torque_HL, Torque_FR, Torque_FL;
    Matrix3d jacobian;
    int LR, idx = 0;


    if(sharedData->idx_legcon[idx])
    {
        LR = 2*(idx%2)-1;
        JointPos = Vector3d(QuadSensor.Encoder.pos.HRR, QuadSensor.Encoder.pos.HRP, QuadSensor.Encoder.pos.HRK);
        JointVel = Vector3d(QuadSensor.Encoder.vel.HRR, QuadSensor.Encoder.vel.HRP, QuadSensor.Encoder.vel.HRK);
        FootPos = Kinematics.FK_OneLeg_local(JointPos, LR);
//        FootPos = QuadState.foot.HR.pos;
        FootPosRef = Vector3d(sharedData->pf_ref[idx*3+0],sharedData->pf_ref[idx*3+1],sharedData->pf_ref[idx*3+2]);
        FootPosRef_local = Kinematics.FK_OneLeg_localize(QuadState, FootPosRef, idx);
        jacobian = Kinematics.getOneLegJacobian(JointPos, LR);
        FootVel = jacobian*JointVel;
        Torque_HR = jacobian.transpose()*(Kp_mat*(FootPosRef_local-FootPos) + Kd_mat*(-FootVel));
        JointPos_IK = Kinematics.IK_OneLeg(FootPos, LR);
        for(int i=0; i<3; i++) {sharedData->joint_IK[idx*3+i]=JointPos_IK[i];}
    }
    else{
        Torque_HR = Vector3d::Zero();
    }
    idx++;

    if(sharedData->idx_legcon[idx])
    {
        LR = 2*(idx%2)-1;
        JointPos = Vector3d(QuadSensor.Encoder.pos.HLR, QuadSensor.Encoder.pos.HLP, QuadSensor.Encoder.pos.HLK);
        JointVel = Vector3d(QuadSensor.Encoder.vel.HLR, QuadSensor.Encoder.vel.HLP, QuadSensor.Encoder.vel.HLK);
        FootPos = Kinematics.FK_OneLeg_local(JointPos, LR);
//        FootPos = QuadState.foot.HL.pos;
        FootPosRef = Vector3d(sharedData->pf_ref[idx*3+0],sharedData->pf_ref[idx*3+1],sharedData->pf_ref[idx*3+2]);
        FootPosRef_local = Kinematics.FK_OneLeg_localize(QuadState, FootPosRef, idx);
        jacobian = Kinematics.getOneLegJacobian(JointPos, LR);
        FootVel = jacobian*JointVel;
        Torque_HL = jacobian.transpose()*(Kp_mat*(FootPosRef_local-FootPos) + Kd_mat*(-FootVel));
        JointPos_IK = Kinematics.IK_OneLeg(FootPos, LR);
        for(int i=0; i<3; i++) {sharedData->joint_IK[idx*3+i]=JointPos_IK[i];}
    }
    else{
        Torque_HL = Vector3d::Zero();
    }
    idx++;

    if(sharedData->idx_legcon[idx])
    {
        LR = 2*(idx%2)-1;
        JointPos = Vector3d(QuadSensor.Encoder.pos.FRR, QuadSensor.Encoder.pos.FRP, QuadSensor.Encoder.pos.FRK);
        JointVel = Vector3d(QuadSensor.Encoder.vel.FRR, QuadSensor.Encoder.vel.FRP, QuadSensor.Encoder.vel.FRK);
        FootPos = Kinematics.FK_OneLeg_local(JointPos, LR);
//        FootPos = QuadState.foot.FR.pos;
        FootPosRef = Vector3d(sharedData->pf_ref[idx*3+0],sharedData->pf_ref[idx*3+1],sharedData->pf_ref[idx*3+2]);
        FootPosRef_local = Kinematics.FK_OneLeg_localize(QuadState, FootPosRef, idx);
        jacobian = Kinematics.getOneLegJacobian(JointPos, LR);
        FootVel = jacobian*JointVel;
        Torque_FR = jacobian.transpose()*(Kp_mat*(FootPosRef_local-FootPos) + Kd_mat*(-FootVel));
        JointPos_IK = Kinematics.IK_OneLeg(FootPos, LR);
        for(int i=0; i<3; i++) {sharedData->joint_IK[idx*3+i]=JointPos_IK[i];}
    }
    else{
        Torque_FR = Vector3d::Zero();
    }
    idx++;

    if(sharedData->idx_legcon[idx])
    {
        LR = 2*(idx%2)-1;
        JointPos = Vector3d(QuadSensor.Encoder.pos.FLR, QuadSensor.Encoder.pos.FLP, QuadSensor.Encoder.pos.FLK);
        JointVel = Vector3d(QuadSensor.Encoder.vel.FLR, QuadSensor.Encoder.vel.FLP, QuadSensor.Encoder.vel.FLK);
        FootPos = Kinematics.FK_OneLeg_local(JointPos, LR);
//        FootPos = QuadState.foot.FL.pos;
        FootPosRef = Vector3d(sharedData->pf_ref[idx*3+0],sharedData->pf_ref[idx*3+1],sharedData->pf_ref[idx*3+2]);
        FootPosRef_local = Kinematics.FK_OneLeg_localize(QuadState, FootPosRef, idx);
        jacobian = Kinematics.getOneLegJacobian(JointPos, LR);
        FootVel = jacobian*JointVel;
        Torque_FL = jacobian.transpose()*(Kp_mat*(FootPosRef_local-FootPos) + Kd_mat*(-FootVel));
        JointPos_IK = Kinematics.IK_OneLeg(FootPos, LR);
        for(int i=0; i<3; i++) {sharedData->joint_IK[idx*3+i]=JointPos_IK[i];}
    }
    else{
        Torque_FL = Vector3d::Zero();
    }

    TorqueRef.HRR = Torque_HR[0];
    TorqueRef.HRP = Torque_HR[1];
    TorqueRef.HRK = Torque_HR[2];
    TorqueRef.HLR = Torque_HL[0];
    TorqueRef.HLP = Torque_HL[1];
    TorqueRef.HLK = Torque_HL[2];
    TorqueRef.FRR = Torque_FR[0];
    TorqueRef.FRP = Torque_FR[1];
    TorqueRef.FRK = Torque_FR[2];
    TorqueRef.FLR = Torque_FL[0];
    TorqueRef.FLP = Torque_FL[1];
    TorqueRef.FLK = Torque_FL[2];
    CurRef = Torque2Current(TorqueRef);



    JointRef = QuadSensor_old.Encoder.pos; //test
    cnt_motion++;
    return  JointRef;
}
