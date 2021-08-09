#include "func_uiuk.h"
extern hubodog5_general Quad;
extern RBMotorController   _DEV_MC[MAX_MC];
extern RBCAN    *canHandler;
extern pRBCORE_SHM sharedData;

extern int MY_CONTROL_FB_GAIN[12];
extern int MY_CONTROL_FF_GAIN[12];

double dt = 0.002;

Vector3d MovePos(double t_now, double t_end, Vector3d pos_start, Vector3d pos_end) {
    Vector3d pf_temp;
    Vector3d pos_temp;

    pf_temp = bezier_calc_3rd(t_now, t_end, Vector3d(pos_start.x(),  0,  0), Vector3d(pos_end.x(), 0, 0));
    pos_temp.x() = pf_temp[0];
    pf_temp = bezier_calc_3rd(t_now, t_end, Vector3d(pos_start.y(),  0,  0), Vector3d(pos_end.y(), 0, 0));
    pos_temp.y() = pf_temp[0];
    pf_temp = bezier_calc_3rd(t_now, t_end, Vector3d(pos_start.z(),  0,  0), Vector3d(pos_end.z(), 0, 0));
    pos_temp.z() = pf_temp[0];

    return pos_temp;
}

Vector3d StanceLeg(double t_now, double t_end, Vector3d pos_start, Vector3d pos_end) {
    Vector3d pf_temp;
    Vector3d pos_temp;

    pf_temp = bezier_calc_3rd(t_now, t_end, Vector3d(pos_start.x(),  0,  0), Vector3d(pos_end.x(), 0, 0));
    pos_temp.x() = pf_temp[0];
    pf_temp = bezier_calc_3rd(t_now, t_end, Vector3d(pos_start.y(),  0,  0), Vector3d(pos_end.y(), 0, 0));
    pos_temp.y() = pf_temp[0];
    pf_temp = bezier_calc_3rd(t_now, t_end, Vector3d(pos_start.z(),  0,  0), Vector3d(pos_end.z(), 0, 0));
    pos_temp.z() = pf_temp[0];

    return pos_temp;
}
Vector3d SwingLeg(double t_now, double t_end, Vector3d pos_start, Vector3d pos_end, double step_z) {
    Vector3d pos_step = pos_end - pos_start;
    double offset_ratio = 0.25;
    Vector3d pf_temp;
    Vector3d pos_temp;

    if(t_now < t_end/4) {
        pf_temp = bezier_calc_3rd(t_now, t_end/4, Vector3d(pos_start.x(), 0, 0), Vector3d(pos_start.x()-offset_ratio*pos_step.x(), 0, 0));
        pos_temp.x() = pf_temp[0];
        pf_temp = bezier_calc_3rd(t_now, t_end/4, Vector3d(pos_start.y(), 0,  0), Vector3d(pos_start.y()-offset_ratio*pos_step.y(), 0, 0));
        pos_temp.y() = pf_temp[0];
    }
    else if((t_now >= t_end/4)&&(t_now < t_end*3/4)) {
        pf_temp = bezier_calc_3rd(t_now-t_end/4, t_end*3/4-t_end/4, Vector3d(pos_start.x()-offset_ratio*pos_step.x(),  0,  0), Vector3d(pos_end.x()+offset_ratio*pos_step.x(), 0, 0));
        pos_temp.x() = pf_temp[0];
        pf_temp = bezier_calc_3rd(t_now-t_end/4, t_end*3/4-t_end/4, Vector3d(pos_start.y()-offset_ratio*pos_step.y(),  0,  0), Vector3d(pos_end.y()+offset_ratio*pos_step.y(), 0, 0));
        pos_temp.y() = pf_temp[0];
    }
    else {
        pf_temp = bezier_calc_3rd(t_now-t_end*3/4, t_end-t_end*3/4, Vector3d(pos_end.x()+offset_ratio*pos_step.x(),  0,  0), Vector3d(pos_end.x(), 0, 0));
        pos_temp.x() = pf_temp[0];
        pf_temp = bezier_calc_3rd(t_now-t_end*3/4, t_end-t_end*3/4, Vector3d(pos_end.y()+offset_ratio*pos_step.y(),  0,  0), Vector3d(pos_end.y(), 0, 0));
        pos_temp.y() = pf_temp[0];
    }
    // z aixs
    if(t_now < t_end/2) {
        pf_temp = bezier_calc_3rd(t_now, t_end/2, Vector3d(pos_start.z(),  0,  0), Vector3d(pos_start.z()+step_z, 0, 0));
        pos_temp.z() = pf_temp[0];
    }
    else {
        pf_temp = bezier_calc_3rd(t_now-t_end/2, t_end-t_end/2, Vector3d(pos_start.z()+step_z,  0,  0), Vector3d(pos_end.z(), 0, 0));
        pos_temp.z() = pf_temp[0];
    }
    //
    return pos_temp;
}

Vector3d StanceLegV(double t_now, double t_end, Vector3d pos_start, Vector3d pos_end, Vector3d vel_start, Vector3d vel_end) {
    Vector3d pf_temp;
    Vector3d pos_temp;

    pf_temp = bezier_calc_3rd(t_now, t_end, Vector3d(pos_start.x(),  vel_start.x(),  0), Vector3d(pos_end.x(), vel_end.x(), 0));
    pos_temp.x() = pf_temp[0];
    printf("ST, t_now:%.3f, t_end:%.3f, pos:%.4f, vel:%.4f\n", t_now, t_end, pf_temp[0], pf_temp[1]);
    pf_temp = bezier_calc_3rd(t_now, t_end, Vector3d(pos_start.y(),  vel_start.y(),  0), Vector3d(pos_end.y(), vel_end.y(), 0));
    pos_temp.y() = pf_temp[0];
    pf_temp = bezier_calc_3rd(t_now, t_end, Vector3d(pos_start.z(),  0,  0), Vector3d(pos_end.z(), 0, 0));
    pos_temp.z() = pf_temp[0];

    return pos_temp;
}
Vector3d SwingLegV(double t_now, double t_end, Vector3d pos_start, Vector3d pos_end, Vector3d vel_start, Vector3d vel_end, double step_z) {
    Vector3d pos_step = pos_end - pos_start;
    double offset_ratio = 0.25;
    Vector3d pf_temp;
    Vector3d pos_temp;

    if(t_now < t_end/4) {
        pf_temp = bezier_calc_3rd(t_now, t_end/4, Vector3d(pos_start.x(), vel_start.x()/4, 0), Vector3d(pos_start.x()-offset_ratio*pos_step.x(), 0, 0));
        pos_temp.x() = pf_temp[0];
        printf("SW, t_now:%.3f, t_end:%.3f, pos:%.4f, vel:%.4f\n", t_now, t_end, pf_temp[0], pf_temp[1]);
        pf_temp = bezier_calc_3rd(t_now, t_end/4, Vector3d(pos_start.y(), vel_start.y()/4,  0), Vector3d(pos_start.y()-offset_ratio*pos_step.y(), 0, 0));
        pos_temp.y() = pf_temp[0];
    }
    else if((t_now >= t_end/4)&&(t_now < t_end*3/4)) {
        pf_temp = bezier_calc_3rd(t_now-t_end/4, t_end*3/4-t_end/4, Vector3d(pos_start.x()-offset_ratio*pos_step.x(),  0,  0), Vector3d(pos_end.x()+offset_ratio*pos_step.x(), 0, 0));
        pos_temp.x() = pf_temp[0];
        printf("SW, t_now:%.3f, t_end:%.3f, pos:%.4f, vel:%.4f\n", t_now, t_end, pf_temp[0], pf_temp[1]);
        pf_temp = bezier_calc_3rd(t_now-t_end/4, t_end*3/4-t_end/4, Vector3d(pos_start.y()-offset_ratio*pos_step.y(),  0,  0), Vector3d(pos_end.y()+offset_ratio*pos_step.y(), 0, 0));
        pos_temp.y() = pf_temp[0];
    }
    else {
        pf_temp = bezier_calc_3rd(t_now-t_end*3/4, t_end-t_end*3/4, Vector3d(pos_end.x()+offset_ratio*pos_step.x(),  0,  0), Vector3d(pos_end.x(), vel_end.x()/4, 0));
        pos_temp.x() = pf_temp[0];
        printf("SW, t_now:%.3f, t_end:%.3f, pos:%.4f, vel:%.4f\n", t_now, t_end, pf_temp[0], pf_temp[1]);
        pf_temp = bezier_calc_3rd(t_now-t_end*3/4, t_end-t_end*3/4, Vector3d(pos_end.y()+offset_ratio*pos_step.y(),  0,  0), Vector3d(pos_end.y(), vel_end.y()/4, 0));
        pos_temp.y() = pf_temp[0];
    }
    // z aixs
    if(t_now < t_end/2) {
        pf_temp = bezier_calc_3rd(t_now, t_end/2, Vector3d(pos_start.z(),  0,  0), Vector3d(pos_start.z()+step_z, 0, 0));
        pos_temp.z() = pf_temp[0];
    }
    else {
        pf_temp = bezier_calc_3rd(t_now-t_end/2, t_end-t_end/2, Vector3d(pos_start.z()+step_z,  0,  0), Vector3d(pos_end.z(), 0, 0));
        pos_temp.z() = pf_temp[0];
    }
    //


    return pos_temp;
}

Vector3d SwingLeg_J(double t_now, double t_end, Vector3d pos_start, Vector3d pos_end, double step_z, double t_jump, double ofs_jump) {
    Vector3d pos_step = pos_end - pos_start;
    double offset_ratio = 0.25;
    Vector3d pf_temp;
    Vector3d pos_temp;

    if(t_now < t_end/4) {
        pf_temp = bezier_calc_3rd(t_now, t_end/4, Vector3d(pos_start.x(), 0, 0), Vector3d(pos_start.x()-offset_ratio*pos_step.x(), 0, 0));
        pos_temp.x() = pf_temp[0];
        pf_temp = bezier_calc_3rd(t_now, t_end/4, Vector3d(pos_start.y(),  0,  0), Vector3d(pos_start.y()-offset_ratio*pos_step.y(), 0, 0));
        pos_temp.y() = pf_temp[0];
    }
    else if((t_now >= t_end/4)&&(t_now < t_end*3/4)) {
        pf_temp = bezier_calc_3rd(t_now-t_end/4, t_end*3/4-t_end/4, Vector3d(pos_start.x()-offset_ratio*pos_step.x(),  0,  0), Vector3d(pos_end.x()+offset_ratio*pos_step.x(), 0, 0));
        pos_temp.x() = pf_temp[0];
        pf_temp = bezier_calc_3rd(t_now-t_end/4, t_end*3/4-t_end/4, Vector3d(pos_start.y()-offset_ratio*pos_step.y(),  0,  0), Vector3d(pos_end.y()+offset_ratio*pos_step.y(), 0, 0));
        pos_temp.y() = pf_temp[0];
    }
    else {
        pf_temp = bezier_calc_3rd(t_now-t_end*3/4, t_end-t_end*3/4, Vector3d(pos_end.x()+offset_ratio*pos_step.x(),  0,  0), Vector3d(pos_end.x(), 0, 0));
        pos_temp.x() = pf_temp[0];
        pf_temp = bezier_calc_3rd(t_now-t_end*3/4, t_end-t_end*3/4, Vector3d(pos_end.y()+offset_ratio*pos_step.y(),  0,  0), Vector3d(pos_end.y(), 0, 0));
        pos_temp.y() = pf_temp[0];
    }
    // z aixs
    //double t_swing = t_end-t_jump;
    if(t_now < t_jump) {
        pf_temp = bezier_calc_3rd(t_now, t_jump, Vector3d(pos_start.z(),  0,  0), Vector3d(pos_start.z()+ofs_jump, 0, 0));
        pos_temp.z() = pf_temp[0];
    }
    else if(t_now >= t_jump && t_now < t_end/2) {
        pf_temp = bezier_calc_3rd(t_now-t_jump,  t_end/2-t_jump, Vector3d(pos_start.z()+ofs_jump,  0,  0), Vector3d(pos_start.z()+step_z, 0, 0));
        pos_temp.z() = pf_temp[0];
    }
    else {
        pf_temp = bezier_calc_3rd(t_now-t_end/2, t_end-t_end/2, Vector3d(pos_start.z()+step_z,  0,  0), Vector3d(pos_end.z(), 0, 0));
        pos_temp.z() = pf_temp[0];
    }
    //
    return pos_temp;
}



Vector3d FootXY_Swing_trajectory(double _t_foot_now, double _real_t_step, double _x_start, double _x_end, double _touch_vel, Vector3d _X_dX_ddX_Foot_old){
    double _dsp_ratio = 0;
    double t_toeoff = 0.25*_real_t_step*(1.0 - _dsp_ratio);
    double t_forward = 0.5*_real_t_step*(1.0 - _dsp_ratio);
    double t_touchdown = 0.25*_real_t_step*(1.0 - _dsp_ratio);
    double t_half_dsp = _real_t_step*_dsp_ratio/2.0;

    double step_length = _x_end - _x_start;

    double p_toeoff = _x_start + 0.25*_x_start;
    double p_forward = _x_end + step_length*0.25;
    double p_touchdown = _x_end;

    if(_t_foot_now < t_half_dsp - 0.5*dt)
    {
    }
    else if(_t_foot_now < t_half_dsp + t_toeoff - 0.5*dt){

        Vector3d p_dp_ddp = calc_3rd(_t_foot_now - dt, t_half_dsp + t_toeoff, _X_dX_ddX_Foot_old, Vector3d(p_toeoff,0,0), dt);

        return p_dp_ddp;
    }
    else if(_t_foot_now < t_half_dsp + t_toeoff + t_forward - 0.5*dt){             //foot rising, 3th order trajectory up

        Vector3d p_dp_ddp = calc_3rd(_t_foot_now - dt, t_half_dsp + t_toeoff + t_forward, _X_dX_ddX_Foot_old, Vector3d(p_forward,0,0), dt);

        return p_dp_ddp; //return z, dz, ddz
    }
    else if(_t_foot_now < t_half_dsp + t_toeoff + t_forward + t_touchdown - 0.5*dt){
        Vector3d p_dp_ddp = calc_3rd(_t_foot_now - dt, t_half_dsp + t_toeoff + t_forward + t_touchdown, _X_dX_ddX_Foot_old, Vector3d(p_touchdown,_touch_vel,0), dt);

        return p_dp_ddp;
    }
    else
    {
    }

    return _X_dX_ddX_Foot_old;
}


Vector3d FootXY_trajectory(double _t_foot_now, double _real_t_step, double _x_start, double _x_end, double _off_vel, Vector3d _X_dX_ddX_Foot_old){
    double t_forward = _real_t_step;


    Vector3d p_dp_ddp = calc_3rd(_t_foot_now-dt, t_forward, _X_dX_ddX_Foot_old, Vector3d(_x_end,_off_vel,0), dt);

    return p_dp_ddp;

}


Vector3d FootZ_Swing_trajectory(double _t_foot_now, double _real_t_step, double _z_start, double _z_end, double _step_z, Vector3d _Z_dZ_ddZ_Foot_old){
    double t_up  = _real_t_step*0.5;
    double t_down = _real_t_step*0.5;

     if(_t_foot_now < t_up - 0.5*dt){             //foot rising, 3th order trajectory up

        Vector3d p_dp_ddp = calc_3rd(_t_foot_now-dt, t_up, _Z_dZ_ddZ_Foot_old, Vector3d(_z_start + _step_z,0,0), dt);

        return p_dp_ddp;//vec3(zFoot, dzFoot, ddzFoot); //return z, dz, ddz

    }
    else{                  // foot down, 3th order trajectory down

        Vector3d p_dp_ddp = calc_3rd(_t_foot_now-dt, t_up + t_down, _Z_dZ_ddZ_Foot_old, Vector3d(_z_end,0, 0), dt);

        return p_dp_ddp;//vec3(zFoot, dzFoot, ddzFoot);  //return z, dz, ddz
    }


    return _Z_dZ_ddZ_Foot_old;
}

Vector3d FootZ_Swing_Jump_trajectory(double _t_foot_now, double _real_t_step, double _z_start, double _z_end, double _step_z, Vector3d _Z_dZ_ddZ_Foot_old){
    double t_up  = _real_t_step*0.5;
    double t_down = _real_t_step*0.5;

     if(_t_foot_now < t_up - 0.5*dt){             //foot rising, 3th order trajectory up

        Vector3d p_dp_ddp = calc_3rd(_t_foot_now-dt, t_up, _Z_dZ_ddZ_Foot_old, Vector3d(_z_start + _step_z,0,0), dt);

        return p_dp_ddp;//vec3(zFoot, dzFoot, ddzFoot); //return z, dz, ddz

    }
    else{                  // foot down, 3th order trajectory down

        Vector3d p_dp_ddp = calc_3rd(_t_foot_now-dt, t_up + t_down, _Z_dZ_ddZ_Foot_old, Vector3d(_z_end,-1.0, 0), dt);

        return p_dp_ddp;//vec3(zFoot, dzFoot, ddzFoot);  //return z, dz, ddz
    }


    return _Z_dZ_ddZ_Foot_old;
}

Vector3d FootZ_Stance_Jump_trajectory(double _t_foot_now, double _real_t_step, double _z_start, double _z_push_vel, Vector3d _Z_dZ_ddZ_Foot_old){
    double t_landing  = _real_t_step*0.5;
    double t_push = _real_t_step*0.5;

     if(_t_foot_now < t_landing - 0.5*dt){             //foot rising, 3th order trajectory up

        Vector3d p_dp_ddp = calc_3rd(_t_foot_now-dt, t_landing, _Z_dZ_ddZ_Foot_old, Vector3d(_z_start+0.008,0,0), dt);

        return p_dp_ddp;//vec3(zFoot, dzFoot, ddzFoot); //return z, dz, ddz

    }
    else{                  // foot down, 3th order trajectory down

        Vector3d p_dp_ddp = calc_3rd(_t_foot_now-dt, t_landing + t_push, _Z_dZ_ddZ_Foot_old, Vector3d(_z_start,_z_push_vel,0), dt);

        return p_dp_ddp;//vec3(zFoot, dzFoot, ddzFoot);  //return z, dz, ddz
    }


    return _Z_dZ_ddZ_Foot_old;
}

void hubodog5_general::Trot_Trajectory() {

}

Vector3d FootStepRotation(double step_r, int lnum) {
    Vector3d offset;
    if(lnum == HR)      offset = Quad.Kinematics.offset_p2rh + Quad.Kinematics.offset_rh2rp;
    else if(lnum == HL) offset = Quad.Kinematics.offset_p2lh + Quad.Kinematics.offset_lh2lp;
    else if(lnum == FR) offset = Quad.Kinematics.offset_p2rs + Quad.Kinematics.offset_rs2rp;
    else if(lnum == FL) offset = Quad.Kinematics.offset_p2ls + Quad.Kinematics.offset_ls2lp;

    Vector3d step_rotation = Vector3d(offset.x()-offset.x()*cos(step_r) + offset.y()*sin(step_r), -offset.x()*sin(step_r) + offset.y()-offset.y()*cos(step_r), 0);
    return step_rotation;
}

Vector3d ComYawCompensation(double angle, int lnum) {
    Vector3d offset;
    if(lnum == HR)      offset = Quad.Kinematics.offset_p2rh;
    else if(lnum == HL) offset = Quad.Kinematics.offset_p2lh;
    else if(lnum == FR) offset = Quad.Kinematics.offset_p2rs;
    else if(lnum == FL) offset = Quad.Kinematics.offset_p2ls;

    Vector3d Compensation = Vector3d(-offset.x()+offset.x()*cos(angle) - offset.y()*sin(angle), offset.x()*sin(angle) - offset.y()+offset.y()*cos(angle), 0);
    return Compensation;
}

void BackFlipMotion() {
    QuadJoint QJ;
    double t_target;

    for(int i=0;i<NO_OF_JOINTS;i++)
    {
        MY_CONTROL_FB_GAIN[i] = 10;
        MY_CONTROL_FF_GAIN[i] = 0;
    }

    if(sharedData->RaisimSimulation == true) {
        QJ.HRR = 0;
        QJ.HLR = 0;
        QJ.FRR = 0;
        QJ.FLR = 0;

        // 1. down
        // hind
        QJ.HRP = 25;
        QJ.HLP = 25;
        QJ.HRK = -110;
        QJ.HLK = -110;

        //front
        QJ.FRP = 25;
        QJ.FLP = 25;
        QJ.FRK = -110;
        QJ.FLK = -110;

        t_target = 1000;

        _DEV_MC[HRR].RBJoint_SetMoveJoint(QJ.HRR, t_target, MOVE_ABSOLUTE);
        _DEV_MC[HRP].RBJoint_SetMoveJoint(QJ.HRP, t_target, MOVE_ABSOLUTE);
        _DEV_MC[HRK].RBJoint_SetMoveJoint(QJ.HRK, t_target, MOVE_ABSOLUTE);
        _DEV_MC[HLR].RBJoint_SetMoveJoint(QJ.HLR, t_target, MOVE_ABSOLUTE);
        _DEV_MC[HLP].RBJoint_SetMoveJoint(QJ.HLP, t_target, MOVE_ABSOLUTE);
        _DEV_MC[HLK].RBJoint_SetMoveJoint(QJ.HLK, t_target, MOVE_ABSOLUTE);
        _DEV_MC[FRR].RBJoint_SetMoveJoint(QJ.FRR, t_target, MOVE_ABSOLUTE);
        _DEV_MC[FRP].RBJoint_SetMoveJoint(QJ.FRP, t_target, MOVE_ABSOLUTE);
        _DEV_MC[FRK].RBJoint_SetMoveJoint(QJ.FRK, t_target, MOVE_ABSOLUTE);
        _DEV_MC[FLR].RBJoint_SetMoveJoint(QJ.FLR, t_target, MOVE_ABSOLUTE);
        _DEV_MC[FLP].RBJoint_SetMoveJoint(QJ.FLP, t_target, MOVE_ABSOLUTE);
        _DEV_MC[FLK].RBJoint_SetMoveJoint(QJ.FLK, t_target, MOVE_ABSOLUTE);
        usleep(t_target * 1e3);

        // 2. push
        // hind
        QJ.HRP = 30;
        QJ.HLP = 30;
        QJ.HRK = -30;
        QJ.HLK = -30;

        // front
        QJ.FRP = 20;
        QJ.FLP = 20;
        QJ.FRK = -30;
        QJ.FLK = -30;

        t_target = 55; // loger time more rotation

        _DEV_MC[HRR].RBJoint_SetMoveJoint(QJ.HRR, t_target, MOVE_ABSOLUTE);
        _DEV_MC[HRP].RBJoint_SetMoveJoint(QJ.HRP, t_target, MOVE_ABSOLUTE);
        _DEV_MC[HRK].RBJoint_SetMoveJoint(QJ.HRK, t_target, MOVE_ABSOLUTE);
        _DEV_MC[HLR].RBJoint_SetMoveJoint(QJ.HLR, t_target, MOVE_ABSOLUTE);
        _DEV_MC[HLP].RBJoint_SetMoveJoint(QJ.HLP, t_target, MOVE_ABSOLUTE);
        _DEV_MC[HLK].RBJoint_SetMoveJoint(QJ.HLK, t_target, MOVE_ABSOLUTE);
        _DEV_MC[FRR].RBJoint_SetMoveJoint(QJ.FRR, t_target, MOVE_ABSOLUTE);
        _DEV_MC[FRP].RBJoint_SetMoveJoint(QJ.FRP, t_target, MOVE_ABSOLUTE);
        _DEV_MC[FRK].RBJoint_SetMoveJoint(QJ.FRK, t_target, MOVE_ABSOLUTE);
        _DEV_MC[FLR].RBJoint_SetMoveJoint(QJ.FLR, t_target, MOVE_ABSOLUTE);
        _DEV_MC[FLP].RBJoint_SetMoveJoint(QJ.FLP, t_target, MOVE_ABSOLUTE);
        _DEV_MC[FLK].RBJoint_SetMoveJoint(QJ.FLK, t_target, MOVE_ABSOLUTE);
        usleep(t_target * 1e3);

        // 3. hind rotate and push
        // hind
        QJ.HRP = 130;
        QJ.HLP = 130;
        QJ.HRK = -30;
        QJ.HLK = -30;

        // front
        QJ.FRP = 20;
        QJ.FLP = 20;
        QJ.FRK = -30;
        QJ.FLK = -30;

        t_target = 400;

        _DEV_MC[HRR].RBJoint_SetMoveJoint(QJ.HRR, t_target, MOVE_ABSOLUTE);
        _DEV_MC[HRP].RBJoint_SetMoveJoint(QJ.HRP, t_target, MOVE_ABSOLUTE);
        _DEV_MC[HRK].RBJoint_SetMoveJoint(QJ.HRK, t_target, MOVE_ABSOLUTE);
        _DEV_MC[HLR].RBJoint_SetMoveJoint(QJ.HLR, t_target, MOVE_ABSOLUTE);
        _DEV_MC[HLP].RBJoint_SetMoveJoint(QJ.HLP, t_target, MOVE_ABSOLUTE);
        _DEV_MC[HLK].RBJoint_SetMoveJoint(QJ.HLK, t_target, MOVE_ABSOLUTE);
        _DEV_MC[FRR].RBJoint_SetMoveJoint(QJ.FRR, t_target, MOVE_ABSOLUTE);
        _DEV_MC[FRP].RBJoint_SetMoveJoint(QJ.FRP, t_target, MOVE_ABSOLUTE);
        _DEV_MC[FRK].RBJoint_SetMoveJoint(QJ.FRK, t_target, MOVE_ABSOLUTE);
        _DEV_MC[FLR].RBJoint_SetMoveJoint(QJ.FLR, t_target, MOVE_ABSOLUTE);
        _DEV_MC[FLP].RBJoint_SetMoveJoint(QJ.FLP, t_target, MOVE_ABSOLUTE);
        _DEV_MC[FLK].RBJoint_SetMoveJoint(QJ.FLK, t_target, MOVE_ABSOLUTE);
        usleep(t_target * 1e3);

        // 4. return to walk ready
        // hind
        QJ.HRP = 60;//-10;
        QJ.HLP = 60;//-10;
        QJ.HRK = -110;//+50;
        QJ.HLK = -110;//+50;

        // front
        QJ.FRP = 60;//-10;
        QJ.FLP = 60;//-10;
        QJ.FRK = -110;//+50;
        QJ.FLK = -110;//+50;

        t_target = 300;

        _DEV_MC[HRR].RBJoint_SetMoveJoint(QJ.HRR, t_target, MOVE_ABSOLUTE);
        _DEV_MC[HRP].RBJoint_SetMoveJoint(QJ.HRP, t_target, MOVE_ABSOLUTE);
        _DEV_MC[HRK].RBJoint_SetMoveJoint(QJ.HRK, t_target, MOVE_ABSOLUTE);
        _DEV_MC[HLR].RBJoint_SetMoveJoint(QJ.HLR, t_target, MOVE_ABSOLUTE);
        _DEV_MC[HLP].RBJoint_SetMoveJoint(QJ.HLP, t_target, MOVE_ABSOLUTE);
        _DEV_MC[HLK].RBJoint_SetMoveJoint(QJ.HLK, t_target, MOVE_ABSOLUTE);
        _DEV_MC[FRR].RBJoint_SetMoveJoint(QJ.FRR, t_target, MOVE_ABSOLUTE);
        _DEV_MC[FRP].RBJoint_SetMoveJoint(QJ.FRP, t_target, MOVE_ABSOLUTE);
        _DEV_MC[FRK].RBJoint_SetMoveJoint(QJ.FRK, t_target, MOVE_ABSOLUTE);
        _DEV_MC[FLR].RBJoint_SetMoveJoint(QJ.FLR, t_target, MOVE_ABSOLUTE);
        _DEV_MC[FLP].RBJoint_SetMoveJoint(QJ.FLP, t_target, MOVE_ABSOLUTE);
        _DEV_MC[FLK].RBJoint_SetMoveJoint(QJ.FLK, t_target, MOVE_ABSOLUTE);
        usleep(t_target * 1e3);

        FILE_LOG(logSUCCESS) << "BACK FLIP SIMULATION Done.";
    }
    else {
        double t_temp = 1;
        QJ.HRR = 0;
        QJ.HLR = 0;
        QJ.FRR = 0;
        QJ.FLR = 0;

        // 1. down
        // hind
        QJ.HRP = (30-5);
        QJ.HLP = (30-5);
        QJ.HRK = -110 - (QJ.HRP-30)/11;
        QJ.HLK = -110 - (QJ.HLP-30)/11;

        //front
        QJ.FRP = (30-5);
        QJ.FLP = (30-5);
        QJ.FRK = -110 - (QJ.FRP-30)/11;
        QJ.FLK = -110 - (QJ.FLP-30)/11;

        t_target = 1000;

        _DEV_MC[HRR].RBJoint_SetMoveJoint(QJ.HRR, t_target, MOVE_ABSOLUTE);
        _DEV_MC[HRP].RBJoint_SetMoveJoint(QJ.HRP, t_target, MOVE_ABSOLUTE);
        _DEV_MC[HRK].RBJoint_SetMoveJoint(QJ.HRK, t_target, MOVE_ABSOLUTE);
        _DEV_MC[HLR].RBJoint_SetMoveJoint(QJ.HLR, t_target, MOVE_ABSOLUTE);
        _DEV_MC[HLP].RBJoint_SetMoveJoint(QJ.HLP, t_target, MOVE_ABSOLUTE);
        _DEV_MC[HLK].RBJoint_SetMoveJoint(QJ.HLK, t_target, MOVE_ABSOLUTE);
        _DEV_MC[FRR].RBJoint_SetMoveJoint(QJ.FRR, t_target, MOVE_ABSOLUTE);
        _DEV_MC[FRP].RBJoint_SetMoveJoint(QJ.FRP, t_target, MOVE_ABSOLUTE);
        _DEV_MC[FRK].RBJoint_SetMoveJoint(QJ.FRK, t_target, MOVE_ABSOLUTE);
        _DEV_MC[FLR].RBJoint_SetMoveJoint(QJ.FLR, t_target, MOVE_ABSOLUTE);
        _DEV_MC[FLP].RBJoint_SetMoveJoint(QJ.FLP, t_target, MOVE_ABSOLUTE);
        _DEV_MC[FLK].RBJoint_SetMoveJoint(QJ.FLK, t_target, MOVE_ABSOLUTE);
        usleep(t_target * 1e3);

        // 2. push
        // hind
        QJ.HRP = 30;
        QJ.HLP = 30;
        QJ.HLK = -20-10 - (QJ.HRP-30)/11;
        QJ.HRK = -20-10 - (QJ.HLP-30)/11;

        // front
        QJ.FRP = 20;
        QJ.FLP = 20;
        QJ.FRK = -20-10 - (QJ.FRP-30)/11;
        QJ.FLK = -20-10 - (QJ.FLP-30)/11;

        t_target = (60-5)*t_temp; // loger time more rotation

        _DEV_MC[HRR].RBJoint_SetMoveJoint(QJ.HRR, t_target, MOVE_ABSOLUTE);
        _DEV_MC[HRP].RBJoint_SetMoveJoint(QJ.HRP, t_target, MOVE_ABSOLUTE);
        _DEV_MC[HRK].RBJoint_SetMoveJoint(QJ.HRK, t_target, MOVE_ABSOLUTE);
        _DEV_MC[HLR].RBJoint_SetMoveJoint(QJ.HLR, t_target, MOVE_ABSOLUTE);
        _DEV_MC[HLP].RBJoint_SetMoveJoint(QJ.HLP, t_target, MOVE_ABSOLUTE);
        _DEV_MC[HLK].RBJoint_SetMoveJoint(QJ.HLK, t_target, MOVE_ABSOLUTE);
        _DEV_MC[FRR].RBJoint_SetMoveJoint(QJ.FRR, t_target, MOVE_ABSOLUTE);
        _DEV_MC[FRP].RBJoint_SetMoveJoint(QJ.FRP, t_target, MOVE_ABSOLUTE);
        _DEV_MC[FRK].RBJoint_SetMoveJoint(QJ.FRK, t_target, MOVE_ABSOLUTE);
        _DEV_MC[FLR].RBJoint_SetMoveJoint(QJ.FLR, t_target, MOVE_ABSOLUTE);
        _DEV_MC[FLP].RBJoint_SetMoveJoint(QJ.FLP, t_target, MOVE_ABSOLUTE);
        _DEV_MC[FLK].RBJoint_SetMoveJoint(QJ.FLK, t_target, MOVE_ABSOLUTE);
        usleep(t_target * 1e3);

        // 3. hind rotate and push
        // hind
        QJ.HRP = 130;
        QJ.HLP = 130;
        QJ.HRK = -20-10 - (QJ.HRP-30)/11;
        QJ.HLK = -20-10 - (QJ.HLP-30)/11;

        // front
        QJ.FRP = 20;
        QJ.FLP = 20;
        QJ.FRK = -20-10 - (QJ.FRP-30)/11;
        QJ.FLK = -20-10 - (QJ.FLP-30)/11;

        t_target = (400)*t_temp;

        _DEV_MC[HRR].RBJoint_SetMoveJoint(QJ.HRR, t_target, MOVE_ABSOLUTE);
        _DEV_MC[HRP].RBJoint_SetMoveJoint(QJ.HRP, t_target, MOVE_ABSOLUTE);
        _DEV_MC[HRK].RBJoint_SetMoveJoint(QJ.HRK, t_target, MOVE_ABSOLUTE);
        _DEV_MC[HLR].RBJoint_SetMoveJoint(QJ.HLR, t_target, MOVE_ABSOLUTE);
        _DEV_MC[HLP].RBJoint_SetMoveJoint(QJ.HLP, t_target, MOVE_ABSOLUTE);
        _DEV_MC[HLK].RBJoint_SetMoveJoint(QJ.HLK, t_target, MOVE_ABSOLUTE);
        _DEV_MC[FRR].RBJoint_SetMoveJoint(QJ.FRR, t_target, MOVE_ABSOLUTE);
        _DEV_MC[FRP].RBJoint_SetMoveJoint(QJ.FRP, t_target, MOVE_ABSOLUTE);
        _DEV_MC[FRK].RBJoint_SetMoveJoint(QJ.FRK, t_target, MOVE_ABSOLUTE);
        _DEV_MC[FLR].RBJoint_SetMoveJoint(QJ.FLR, t_target, MOVE_ABSOLUTE);
        _DEV_MC[FLP].RBJoint_SetMoveJoint(QJ.FLP, t_target, MOVE_ABSOLUTE);
        _DEV_MC[FLK].RBJoint_SetMoveJoint(QJ.FLK, t_target, MOVE_ABSOLUTE);
        usleep(t_target * 1e3);

        // 4. return to walk ready
        // hind

        _DEV_MC[HRR].RBFOC_SetGain_POS(50,0,600, 0);
        _DEV_MC[HRP].RBFOC_SetGain_POS(50,0,600, 0);
        _DEV_MC[HRK].RBFOC_SetGain_POS(50,0,600, 0);
        _DEV_MC[HLR].RBFOC_SetGain_POS(50,0,600, 0);
        _DEV_MC[HLP].RBFOC_SetGain_POS(50,0,600, 0);
        _DEV_MC[HLK].RBFOC_SetGain_POS(50,0,600, 0);
        _DEV_MC[FRR].RBFOC_SetGain_POS(50,0,600, 0);
        _DEV_MC[FRP].RBFOC_SetGain_POS(50,0,600, 0);
        _DEV_MC[FRK].RBFOC_SetGain_POS(50,0,600, 0);
        _DEV_MC[FLR].RBFOC_SetGain_POS(50,0,600, 0);
        _DEV_MC[FLP].RBFOC_SetGain_POS(50,0,600, 0);
        _DEV_MC[FLK].RBFOC_SetGain_POS(50,0,600, 0);

        QJ.HRP = 60;//-10;
        QJ.HLP = 60;//-10;
        QJ.HRK = -110 - (QJ.HRP-30)/11;//+50;
        QJ.HLK = -110 - (QJ.HLP-30)/11;//+50;

        // front
        QJ.FRP = 60;//-10;
        QJ.FLP = 60;//-10;
        QJ.FRK = -110 - (QJ.FRP-30)/11;//+50;
        QJ.FLK = -110 - (QJ.FLP-30)/11;//+50;

        t_target = (300-100)*t_temp;

        _DEV_MC[HRR].RBJoint_SetMoveJoint(QJ.HRR, t_target, MOVE_ABSOLUTE);
        _DEV_MC[HRP].RBJoint_SetMoveJoint(QJ.HRP, t_target, MOVE_ABSOLUTE);
        _DEV_MC[HRK].RBJoint_SetMoveJoint(QJ.HRK, t_target, MOVE_ABSOLUTE);
        _DEV_MC[HLR].RBJoint_SetMoveJoint(QJ.HLR, t_target, MOVE_ABSOLUTE);
        _DEV_MC[HLP].RBJoint_SetMoveJoint(QJ.HLP, t_target, MOVE_ABSOLUTE);
        _DEV_MC[HLK].RBJoint_SetMoveJoint(QJ.HLK, t_target, MOVE_ABSOLUTE);
        _DEV_MC[FRR].RBJoint_SetMoveJoint(QJ.FRR, t_target, MOVE_ABSOLUTE);
        _DEV_MC[FRP].RBJoint_SetMoveJoint(QJ.FRP, t_target, MOVE_ABSOLUTE);
        _DEV_MC[FRK].RBJoint_SetMoveJoint(QJ.FRK, t_target, MOVE_ABSOLUTE);
        _DEV_MC[FLR].RBJoint_SetMoveJoint(QJ.FLR, t_target, MOVE_ABSOLUTE);
        _DEV_MC[FLP].RBJoint_SetMoveJoint(QJ.FLP, t_target, MOVE_ABSOLUTE);
        _DEV_MC[FLK].RBJoint_SetMoveJoint(QJ.FLK, t_target, MOVE_ABSOLUTE);
        usleep(t_target * 1e3);
    }
}
