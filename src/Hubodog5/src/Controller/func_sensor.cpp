#include "func_sensor.h"
#include "ESTIMATOR/estimator.h"


extern RBMotorController   _DEV_MC[MAX_MC];
extern RBCAN    *canHandler;
extern pRBCORE_SHM sharedData;

extern int MY_CONTROL_FB_GAIN[12];
extern int MY_CONTROL_FF_GAIN[12];

extern SensorInfo sensor, sensor_ref;
extern hubodog5_general Quad;

void imu_zero(){
//    FILE_LOG(logSUCCESS) << "IMU ZERO";
//    RBCAN_MB mb;
//    mb.channel = 0;//0
//    mb.id = 5;//5
//    mb.data[0] = 2;
//    mb.data[1] = 1;
//    mb.dlc = 2;
//    canHandler->RBCAN_WriteData(mb);
}

double GetOneJointPos(int idx)
{
    return _DEV_MC[idx].CurrentPosition*D2Rf;
}

double GetOneJointPosRef(int idx)
{
    return _DEV_MC[idx].MoveJoints.RefAngleCurrent*D2Rf;
}

double GetOneJointVel(int idx)
{
    return _DEV_MC[idx].CurrentVelocity*D2Rf;
}

double GetOneJointCurrent(int idx)
{
    return _DEV_MC[idx].MeasuredCurrent*0.001;
}

double h_hip = 0;
QuadJoint GetAllJointPos()
{
    QuadJoint QJ;
    QJ.HRR = GetOneJointPos(HRR);
    QJ.HRP = GetOneJointPos(HRP);
    QJ.HRK = GetOneJointPos(HRK) + (QJ.HRP - h_hip*D2Rf)/11.0;
//    QJ.HRK = GetOneJointPos(HRK);
    QJ.HLR = GetOneJointPos(HLR);
    QJ.HLP = GetOneJointPos(HLP);
    QJ.HLK = GetOneJointPos(HLK) + (QJ.HLP - h_hip*D2Rf)/11.0;
//    QJ.HLK = GetOneJointPos(HLK);
    QJ.FRR = GetOneJointPos(FRR);
    QJ.FRP = GetOneJointPos(FRP);
    QJ.FRK = GetOneJointPos(FRK) + (QJ.FRP - h_hip*D2Rf)/11.0;
//    QJ.FRK = GetOneJointPos(FRK);
    QJ.FLR = GetOneJointPos(FLR);
    QJ.FLP = GetOneJointPos(FLP);
    QJ.FLK = GetOneJointPos(FLK) + (QJ.FLP - h_hip*D2Rf)/11.0;
//    QJ.FLK = GetOneJointPos(FLK);
    return QJ;
}

QuadJoint GetAllJointPosRef()
{
    QuadJoint QJ;
    QJ.HRR = GetOneJointPosRef(HRR);
    QJ.HRP = GetOneJointPosRef(HRP);
    QJ.HRK = GetOneJointPosRef(HRK) + (QJ.HRP - h_hip*D2Rf)/11.0;
//    QJ.HRK = GetOneJointPosRef(HRK);
    QJ.HLR = GetOneJointPosRef(HLR);
    QJ.HLP = GetOneJointPosRef(HLP);
    QJ.HLK = GetOneJointPosRef(HLK) + (QJ.HLP - h_hip*D2Rf)/11.0;
//    QJ.HLK = GetOneJointPosRef(HLK);
    QJ.FRR = GetOneJointPosRef(FRR);
    QJ.FRP = GetOneJointPosRef(FRP);
    QJ.FRK = GetOneJointPosRef(FRK) + (QJ.FRP - h_hip*D2Rf)/11.0;
//    QJ.FRK = GetOneJointPosRef(FRK);
    QJ.FLR = GetOneJointPosRef(FLR);
    QJ.FLP = GetOneJointPosRef(FLP);
    QJ.FLK = GetOneJointPosRef(FLK) + (QJ.FLP - h_hip*D2Rf)/11.0;
//    QJ.FLK = GetOneJointPosRef(FLK);
    return QJ;
}

QuadJoint GetAllJointVel()
{
    QuadJoint QJ;
    QJ.HRR = GetOneJointVel(HRR);
    QJ.HRP = GetOneJointVel(HRP);
    QJ.HRK = GetOneJointVel(HRK) + (QJ.HRP)/11.0;
//    QJ.HRK = GetOneJointVel(HRK);
    QJ.HLR = GetOneJointVel(HLR);
    QJ.HLP = GetOneJointVel(HLP);
    QJ.HLK = GetOneJointVel(HLK) + (QJ.HLP)/11.0;
//    QJ.HLK = GetOneJointVel(HLK);
    QJ.FRR = GetOneJointVel(FRR);
    QJ.FRP = GetOneJointVel(FRP);
    QJ.FRK = GetOneJointVel(FRK) + (QJ.FRP)/11.0;
//    QJ.FRK = GetOneJointVel(FRK);
    QJ.FLR = GetOneJointVel(FLR);
    QJ.FLP = GetOneJointVel(FLP);
    QJ.FLK = GetOneJointVel(FLK) + (QJ.FLP)/11.0;
//    QJ.FLK = GetOneJointVel(FLK);
    return QJ;
}

QuadJoint GetAllJointCurrent()
{
    QuadJoint QJ;
    QJ.HRR = GetOneJointCurrent(HRR);
    QJ.HRP = GetOneJointCurrent(HRP);
    QJ.HRK = GetOneJointCurrent(HRK);
    QJ.HLR = GetOneJointCurrent(HLR);
    QJ.HLP = GetOneJointCurrent(HLP);
    QJ.HLK = GetOneJointCurrent(HLK);
    QJ.FRR = GetOneJointCurrent(FRR);
    QJ.FRP = GetOneJointCurrent(FRP);
    QJ.FRK = GetOneJointCurrent(FRK);
    QJ.FLR = GetOneJointCurrent(FLR);
    QJ.FLP = GetOneJointCurrent(FLP);
    QJ.FLK = GetOneJointCurrent(FLK);
    return QJ;
}

QuadJoint GetAllJointAcc()
{
    QuadJoint QJ;
    // add here
    return QJ;
}

QuadJoint pos_pre;
QuadJoint vel_pre;
void set_ref_data(){
    double dt = Quad.dt_main;

    // Encoder part


    pos_pre = sensor_ref.Encoder.pos;
    sensor_ref.Encoder.pos = GetAllJointPosRef();

    vel_pre = sensor_ref.Encoder.vel;
    sensor_ref.Encoder.vel.HRR = (sensor_ref.Encoder.pos.HRR - pos_pre.HRR)/dt;
    sensor_ref.Encoder.vel.HRP = (sensor_ref.Encoder.pos.HRP - pos_pre.HRP)/dt;
    sensor_ref.Encoder.vel.HRK = (sensor_ref.Encoder.pos.HRK - pos_pre.HRK)/dt;
    sensor_ref.Encoder.vel.HLR = (sensor_ref.Encoder.pos.HLR - pos_pre.HLR)/dt;
    sensor_ref.Encoder.vel.HLP = (sensor_ref.Encoder.pos.HLP - pos_pre.HLP)/dt;
    sensor_ref.Encoder.vel.HLK = (sensor_ref.Encoder.pos.HLK - pos_pre.HLK)/dt;
    sensor_ref.Encoder.vel.FRR = (sensor_ref.Encoder.pos.FRR - pos_pre.FRR)/dt;
    sensor_ref.Encoder.vel.FRP = (sensor_ref.Encoder.pos.FRP - pos_pre.FRP)/dt;
    sensor_ref.Encoder.vel.FRK = (sensor_ref.Encoder.pos.FRK - pos_pre.FRK)/dt;
    sensor_ref.Encoder.vel.FLR = (sensor_ref.Encoder.pos.FLR - pos_pre.FLR)/dt;
    sensor_ref.Encoder.vel.FLP = (sensor_ref.Encoder.pos.FLP - pos_pre.FLP)/dt;
    sensor_ref.Encoder.vel.FLK = (sensor_ref.Encoder.pos.FLK - pos_pre.FLK)/dt;

    sensor_ref.Encoder.acc.HRR = (sensor_ref.Encoder.vel.HRR - vel_pre.HRR)/dt;
    sensor_ref.Encoder.acc.HRP = (sensor_ref.Encoder.vel.HRP - vel_pre.HRP)/dt;
    sensor_ref.Encoder.acc.HRK = (sensor_ref.Encoder.vel.HRK - vel_pre.HRK)/dt;
    sensor_ref.Encoder.acc.HLR = (sensor_ref.Encoder.vel.HLR - vel_pre.HLR)/dt;
    sensor_ref.Encoder.acc.HLP = (sensor_ref.Encoder.vel.HLP - vel_pre.HLP)/dt;
    sensor_ref.Encoder.acc.HLK = (sensor_ref.Encoder.vel.HLK - vel_pre.HLK)/dt;
    sensor_ref.Encoder.acc.FRR = (sensor_ref.Encoder.vel.FRR - vel_pre.FRR)/dt;
    sensor_ref.Encoder.acc.FRP = (sensor_ref.Encoder.vel.FRP - vel_pre.FRP)/dt;
    sensor_ref.Encoder.acc.FRK = (sensor_ref.Encoder.vel.FRK - vel_pre.FRK)/dt;
    sensor_ref.Encoder.acc.FLR = (sensor_ref.Encoder.vel.FLR - vel_pre.FLR)/dt;
    sensor_ref.Encoder.acc.FLP = (sensor_ref.Encoder.vel.FLP - vel_pre.FLP)/dt;
    sensor_ref.Encoder.acc.FLK = (sensor_ref.Encoder.vel.FLK - vel_pre.FLK)/dt;

    // need to modify
//    sensor_ref.Encoder.vel = GetAllJointVel();
//    sensor_ref.Encoder.current = GetAllJointCurrent();
//    sensor_ref.Encoder.acc = GetAllJointAcc();
    Quad.QuadSensorRef = sensor_ref;

    // reference
    sharedData->joint_info_ref[0] =     sensor_ref.Encoder.pos.HRR;
    sharedData->joint_info_ref[1] =     sensor_ref.Encoder.pos.HRP;
    sharedData->joint_info_ref[2] =     sensor_ref.Encoder.pos.HRK;
    sharedData->joint_info_ref[3] =     sensor_ref.Encoder.pos.HLR;
    sharedData->joint_info_ref[4] =     sensor_ref.Encoder.pos.HLP;
    sharedData->joint_info_ref[5] =     sensor_ref.Encoder.pos.HLK;
    sharedData->joint_info_ref[6] =     sensor_ref.Encoder.pos.FRR;
    sharedData->joint_info_ref[7] =     sensor_ref.Encoder.pos.FRP;
    sharedData->joint_info_ref[8] =     sensor_ref.Encoder.pos.FRK;
    sharedData->joint_info_ref[9] =     sensor_ref.Encoder.pos.FLR;
    sharedData->joint_info_ref[10] =    sensor_ref.Encoder.pos.FLP;
    sharedData->joint_info_ref[11] =    sensor_ref.Encoder.pos.FLK;

    sharedData->joint_info_ref[12+0] =     sensor_ref.Encoder.vel.HRR;
    sharedData->joint_info_ref[12+1] =     sensor_ref.Encoder.vel.HRP;
    sharedData->joint_info_ref[12+2] =     sensor_ref.Encoder.vel.HRK;
    sharedData->joint_info_ref[12+3] =     sensor_ref.Encoder.vel.HLR;
    sharedData->joint_info_ref[12+4] =     sensor_ref.Encoder.vel.HLP;
    sharedData->joint_info_ref[12+5] =     sensor_ref.Encoder.vel.HLK;
    sharedData->joint_info_ref[12+6] =     sensor_ref.Encoder.vel.FRR;
    sharedData->joint_info_ref[12+7] =     sensor_ref.Encoder.vel.FRP;
    sharedData->joint_info_ref[12+8] =     sensor_ref.Encoder.vel.FRK;
    sharedData->joint_info_ref[12+9] =     sensor_ref.Encoder.vel.FLR;
    sharedData->joint_info_ref[12+10] =    sensor_ref.Encoder.vel.FLP;
    sharedData->joint_info_ref[12+11] =    sensor_ref.Encoder.vel.FLK;

    sharedData->joint_info_ref[24+0] =     sensor_ref.Encoder.acc.HRR;
    sharedData->joint_info_ref[24+1] =     sensor_ref.Encoder.acc.HRP;
    sharedData->joint_info_ref[24+2] =     sensor_ref.Encoder.acc.HRK;
    sharedData->joint_info_ref[24+3] =     sensor_ref.Encoder.acc.HLR;
    sharedData->joint_info_ref[24+4] =     sensor_ref.Encoder.acc.HLP;
    sharedData->joint_info_ref[24+5] =     sensor_ref.Encoder.acc.HLK;
    sharedData->joint_info_ref[24+6] =     sensor_ref.Encoder.acc.FRR;
    sharedData->joint_info_ref[24+7] =     sensor_ref.Encoder.acc.FRP;
    sharedData->joint_info_ref[24+8] =     sensor_ref.Encoder.acc.FRK;
    sharedData->joint_info_ref[24+9] =     sensor_ref.Encoder.acc.FLR;
    sharedData->joint_info_ref[24+10] =    sensor_ref.Encoder.acc.FLP;
    sharedData->joint_info_ref[24+11] =    sensor_ref.Encoder.acc.FLK;

}

double frq_IMU = 50;
double alpha_IMU = 0.98;
double frq_JointVel = 50;

double roll_temp;
double pitch_temp;

Estimator EST(RT_TIMER_PERIOD_MS / 1000);
Eigen::Vector3d _acc;
Eigen::Vector3d _gyro;

double roll_acc_deg;
double pitch_acc_deg;


void get_sensor_data(){

#ifdef VectorNav

#endif
    // Cheap imu
//    EST.setIMUPIGain(0.04, 0);

//    _acc << -sharedData->IMU[0].AccX,
//            -sharedData->IMU[0].AccY,
//            -sharedData->IMU[0].AccZ;

//    _gyro << sharedData->IMU[0].RollVel*D2Rf,
//            sharedData->IMU[0].PitchVel*D2Rf,
//            sharedData->IMU[0].YawVel*D2Rf;

//    double acc_norm = sqrt(_acc.x()*_acc.x() +_acc.y()*_acc.y() + _acc.z()*_acc.z());

//    if(acc_norm < 1e-6)
//        acc_norm = 1e-6;

//    double acc_x = _acc.x()/acc_norm;
//    double acc_y = _acc.y()/acc_norm;
//    double acc_z = _acc.z()/acc_norm;


//    double ftemp0 = atan2f(acc_y, acc_z + 1e-6);
//    double ftemp1;

//    if(fabs(cos(ftemp0))<1e-6)
//        ftemp1 = atan2f(-acc_x, acc_y/sin(ftemp0));
//    else
//        ftemp1 = atan2f(-acc_x, acc_z/cos(ftemp0));

//    roll_acc_deg = ftemp0*R2Df;
//    pitch_acc_deg = ftemp1*R2Df;

//    sharedData->IMU[0].Roll_AHRS = roll_acc_deg;
//    sharedData->IMU[0].Pitch_AHRS = pitch_acc_deg;

//    if(sharedData->IMU[0].RESET_FLAG == true) {
//        sharedData->IMU[0].RESET_FLAG = false;

//        rpy RPY_ACC = rpy(roll_acc_deg*D2R, pitch_acc_deg*D2R, 0.0);
//        quat quat_ACC = quat(RPY_ACC);

//        Vector4d quat_temp;
//        quat_temp << quat_ACC[0], quat_ACC[1], quat_ACC[2], quat_ACC[3];

//        EST.reset_complementaryFilter(quat_temp);
//    }

//    EST.complementaryFilter(_gyro, _acc);

//    Eigen::Vector4d quat_ = EST.getQuat();

//    double q0 = quat_[0];
//    double q1 = quat_[1];
//    double q2 = quat_[2];
//    double q3 = quat_[3];

//    sharedData->IMU[0].Q[0] = q0;
//    sharedData->IMU[0].Q[1] = q1;
//    sharedData->IMU[0].Q[2] = q2;
//    sharedData->IMU[0].Q[3] = q3;

//    double phi = atan2(2 * (q2*q3 + q0*q1), 1 - 2 * (q1*q1 + q2*q2))*57.2957914;
//    double theta = -1 * asin(2 * (q1*q3 - q0*q2))*57.2957914;
//    double psi = atan2(2 * (q1*q2 + q0*q3), 1 - 2 * (q2*q2 + q3*q3))*57.2957914;

//    sharedData->IMU[0].Roll = phi;
//    sharedData->IMU[0].Pitch = theta;
//    sharedData->IMU[0].Yaw = psi;


    // Vectornav
    // IMU part
    double dt = Quad.dt_main;
    frq_IMU = sharedData->frq_IMUFilter;
    alpha_IMU = sharedData->alpha_IMUFilter;

//    sharedData->IMU[1].Roll = Lowpass_filter(sharedData->IMU[0].Roll, sharedData->IMU[1].Roll, frq_IMU, dt);
//    sharedData->IMU[1].Pitch = Lowpass_filter(sharedData->IMU[0].Pitch, sharedData->IMU[1].Pitch, frq_IMU, dt);
//    sharedData->IMU[1].Yaw = Lowpass_filter(sharedData->IMU[0].Yaw, sharedData->IMU[1].Yaw, frq_IMU, dt);

    sharedData->IMU[1].Roll =  sharedData->IMU[0].Roll;
    sharedData->IMU[1].Pitch = sharedData->IMU[0].Pitch;
    sharedData->IMU[1].Yaw =   sharedData->IMU[0].Yaw;

    double angle_limit = 40.;

    if(sharedData->IMU[1].Roll > angle_limit) {
        sharedData->IMU[1].Roll = angle_limit;
        sharedData->isFalled = true;
    }
    else if(sharedData->IMU[1].Roll < -angle_limit) {
        sharedData->IMU[1].Roll = -angle_limit;
        sharedData->isFalled = true;
    }

    if(sharedData->IMU[1].Pitch > angle_limit) {
        sharedData->IMU[1].Pitch = angle_limit;
        sharedData->isFalled = true;
    }
    else if(sharedData->IMU[1].Pitch < -angle_limit) {
        sharedData->IMU[1].Pitch = -angle_limit;
        sharedData->isFalled = true;
    }


    quat quat_temp = quat(rpy(sharedData->IMU[1].Roll*D2Rf, sharedData->IMU[1].Pitch*D2Rf, sharedData->joy_Orientation[2]));

    //quat quat_temp = quat(rpy(sharedData->joy_Orientation[0], sharedData->joy_Orientation[1], sharedData->joy_Orientation[2]));

//    quat quat_temp = quat(rpy(0, 0, 0));

    for(int i=0; i<4; i++) sharedData->IMU[1].Q[i] = quat_temp[i];

    sharedData->IMU[1].RollVel = Lowpass_filter(sharedData->IMU[0].RollVel, sharedData->IMU[1].RollVel, frq_IMU, dt);
    sharedData->IMU[1].PitchVel = Lowpass_filter(sharedData->IMU[0].PitchVel, sharedData->IMU[1].PitchVel, frq_IMU, dt);
    sharedData->IMU[1].YawVel = Lowpass_filter(sharedData->IMU[0].YawVel, sharedData->IMU[1].YawVel, frq_IMU, dt);

    sharedData->IMU[1].AccX = Lowpass_filter(sharedData->IMU[0].AccX, sharedData->IMU[1].AccX, frq_IMU, dt);
    sharedData->IMU[1].AccY = Lowpass_filter(sharedData->IMU[0].AccY, sharedData->IMU[1].AccY, frq_IMU, dt);
    sharedData->IMU[1].AccZ = Lowpass_filter(sharedData->IMU[0].AccZ, sharedData->IMU[1].AccZ, frq_IMU, dt);

    sharedData->IMU[0].AccPitch = atan(-sharedData->IMU[1].AccX / sqrt(pow(sharedData->IMU[1].AccY, 2) + pow(sharedData->IMU[1].AccZ, 2)))*R2Df;
    sharedData->IMU[0].AccRoll = atan(sharedData->IMU[1].AccY / sqrt(pow(sharedData->IMU[1].AccX, 2) + pow(sharedData->IMU[1].AccZ, 2)))*R2Df;

    sharedData->IMU[0].CFRoll = (alpha_IMU*sharedData->IMU[0].Roll + (1-alpha_IMU)*sharedData->IMU[0].AccRoll);
    sharedData->IMU[0].CFPitch = (alpha_IMU*sharedData->IMU[0].Pitch + (1-alpha_IMU)*sharedData->IMU[0].AccPitch);

    sharedData->IMU[1].CFRoll = Lowpass_filter(sharedData->IMU[0].CFRoll, sharedData->IMU[1].CFRoll, frq_IMU, dt);
    sharedData->IMU[1].CFPitch = Lowpass_filter(sharedData->IMU[0].CFPitch, sharedData->IMU[1].CFPitch, frq_IMU, dt);

//    sharedData->comcon_variable_plot[0] = sharedData->IMU[1].Roll;
//    sharedData->comcon_variable_plot[1] = sharedData->IMU[1].Pitch;
//    sharedData->comcon_variable_plot[2] = sharedData->IMU[1].AccX;
//    sharedData->comcon_variable_plot[3] = sharedData->IMU[1].AccY;
//    sharedData->comcon_variable_plot[4] = 0;
//    sharedData->comcon_variable_plot[5] = 0;



    if(sharedData->btn_IMUFilter == 0) {
//        sensor.IMU.angle = D2Rf*Vector3d(sharedData->IMU[0].Roll,
//                                         sharedData->IMU[0].Pitch,
//                                         sharedData->IMU[0].Yaw);
//        sensor.IMU.quat = Vector4d::Zero();
//        sensor.IMU.quat << sharedData->IMU[0].Q[0],
//                           sharedData->IMU[0].Q[1],
//                           sharedData->IMU[0].Q[2],
//                           sharedData->IMU[0].Q[3];
//        sensor.IMU.vel = D2Rf*Vector3d(sharedData->IMU[0].RollVel,
//                                       sharedData->IMU[0].PitchVel,
//                                       sharedData->IMU[0].YawVel);
//        sensor.IMU.acc = Vector3d(sharedData->IMU[0].AccX,
//                                  sharedData->IMU[0].AccY,
//                                  sharedData->IMU[0].AccZ);
    }
    else {
        sensor.IMU.angle = D2Rf*Vector3d(sharedData->IMU[0].Roll,
                                         sharedData->IMU[0].Pitch,
                                         sharedData->IMU[0].Yaw);
        sensor.IMU.quat = Vector4d::Zero();
        sensor.IMU.quat << sharedData->IMU[1].Q[0],
                           sharedData->IMU[1].Q[1],
                           sharedData->IMU[1].Q[2],
                           sharedData->IMU[1].Q[3];
        sensor.IMU.vel = D2Rf*Vector3d(sharedData->IMU[1].RollVel,
                                       sharedData->IMU[1].PitchVel,
                                       sharedData->IMU[1].YawVel);
        sensor.IMU.acc = Vector3d(sharedData->IMU[1].AccX,
                                  sharedData->IMU[1].AccY,
                                  sharedData->IMU[1].AccZ);

        sensor.IMU.quat_RPY << sharedData->IMU[0].Q[0],
                               sharedData->IMU[0].Q[1],
                               sharedData->IMU[0].Q[2],
                               sharedData->IMU[0].Q[3];
    }

    if(Quad.isSimulation || sharedData->isIMUAlwaysZero)
    {
        sensor.IMU.angle = Vector3d::Zero();
        sensor.IMU.quat = Vector4d::Zero();
        sensor.IMU.quat(0) = 1.0;
        sensor.IMU.vel = Vector3d::Zero();
        sensor.IMU.acc = Vector3d::Zero();
    }

    // Encoder part
    sensor.Encoder.pos = GetAllJointPos();
    //sensor.Encoder.pos_ref = GetAllJointPosRef();
//    sensor.Encoder.vel = GetAllJointVel();

    static QuadJoint vel_temp, vel_temp_filtered;

    vel_temp = GetAllJointVel();

    for(int i=0 ; i<12 ; i++){
        vel_temp_filtered.QJ_Array[i] = Lowpass_filter(vel_temp.QJ_Array[i], vel_temp_filtered.QJ_Array[i], 5, 0.002);
    }


    sensor.Encoder.vel = vel_temp_filtered;


    sensor.Encoder.current = GetAllJointCurrent();
    sensor.Encoder.acc = GetAllJointAcc();

//    if(Quad.isSimulation)
//    {
//        sensor.Encoder.pos = GetAllJointPosRef();
//        cout << "Simulation!" << endl;
//    }

    Quad.QuadSensor = sensor;


    // LPF part
//    Quad.QuadSensor_filtered = Quad.QuadSensor;
    frq_JointVel = sharedData->frq_JointFilter;

    Quad.QuadSensor_filtered.Encoder.vel.HRR = Lowpass_filter(Quad.QuadSensor.Encoder.vel.HRR, Quad.QuadSensor_filtered.Encoder.vel.HRR, frq_JointVel, dt);
    Quad.QuadSensor_filtered.Encoder.vel.HRP = Lowpass_filter(Quad.QuadSensor.Encoder.vel.HRP, Quad.QuadSensor_filtered.Encoder.vel.HRP, frq_JointVel, dt);
    Quad.QuadSensor_filtered.Encoder.vel.HRK = Lowpass_filter(Quad.QuadSensor.Encoder.vel.HRK, Quad.QuadSensor_filtered.Encoder.vel.HRK, frq_JointVel, dt);
    Quad.QuadSensor_filtered.Encoder.vel.HLR = Lowpass_filter(Quad.QuadSensor.Encoder.vel.HLR, Quad.QuadSensor_filtered.Encoder.vel.HLR, frq_JointVel, dt);
    Quad.QuadSensor_filtered.Encoder.vel.HLP = Lowpass_filter(Quad.QuadSensor.Encoder.vel.HLP, Quad.QuadSensor_filtered.Encoder.vel.HLP, frq_JointVel, dt);
    Quad.QuadSensor_filtered.Encoder.vel.HLK = Lowpass_filter(Quad.QuadSensor.Encoder.vel.HLK, Quad.QuadSensor_filtered.Encoder.vel.HLK, frq_JointVel, dt);
    Quad.QuadSensor_filtered.Encoder.vel.FRR = Lowpass_filter(Quad.QuadSensor.Encoder.vel.FRR, Quad.QuadSensor_filtered.Encoder.vel.FRR, frq_JointVel, dt);
    Quad.QuadSensor_filtered.Encoder.vel.FRP = Lowpass_filter(Quad.QuadSensor.Encoder.vel.FRP, Quad.QuadSensor_filtered.Encoder.vel.FRP, frq_JointVel, dt);
    Quad.QuadSensor_filtered.Encoder.vel.FRK = Lowpass_filter(Quad.QuadSensor.Encoder.vel.FRK, Quad.QuadSensor_filtered.Encoder.vel.FRK, frq_JointVel, dt);
    Quad.QuadSensor_filtered.Encoder.vel.FLR = Lowpass_filter(Quad.QuadSensor.Encoder.vel.FLR, Quad.QuadSensor_filtered.Encoder.vel.FLR, frq_JointVel, dt);
    Quad.QuadSensor_filtered.Encoder.vel.FLP = Lowpass_filter(Quad.QuadSensor.Encoder.vel.FLP, Quad.QuadSensor_filtered.Encoder.vel.FLP, frq_JointVel, dt);
    Quad.QuadSensor_filtered.Encoder.vel.FLK = Lowpass_filter(Quad.QuadSensor.Encoder.vel.FLK, Quad.QuadSensor_filtered.Encoder.vel.FLK, frq_JointVel, dt);


    if(sharedData->isJointFilter){
        //Quad.QuadSensor.Encoder.vel = Quad.QuadSensor_filtered.Encoder.vel;
    }

    // shared memory
    // raw data
    sharedData->joint_info[0] =         sensor.Encoder.pos.HRR;
    sharedData->joint_info[1] =         sensor.Encoder.pos.HRP;
    sharedData->joint_info[2] =         sensor.Encoder.pos.HRK;
    sharedData->joint_info[3] =         sensor.Encoder.pos.HLR;
    sharedData->joint_info[4] =         sensor.Encoder.pos.HLP;
    sharedData->joint_info[5] =         sensor.Encoder.pos.HLK;
    sharedData->joint_info[6] =         sensor.Encoder.pos.FRR;
    sharedData->joint_info[7] =         sensor.Encoder.pos.FRP;
    sharedData->joint_info[8] =         sensor.Encoder.pos.FRK;
    sharedData->joint_info[9] =         sensor.Encoder.pos.FLR;
    sharedData->joint_info[10] =        sensor.Encoder.pos.FLP;
    sharedData->joint_info[11] =        sensor.Encoder.pos.FLK;

    sharedData->joint_info[12+0] =      sensor.Encoder.vel.HRR;
    sharedData->joint_info[12+1] =      sensor.Encoder.vel.HRP;
    sharedData->joint_info[12+2] =      sensor.Encoder.vel.HRK;
    sharedData->joint_info[12+3] =      sensor.Encoder.vel.HLR;
    sharedData->joint_info[12+4] =      sensor.Encoder.vel.HLP;
    sharedData->joint_info[12+5] =      sensor.Encoder.vel.HLK;
    sharedData->joint_info[12+6] =      sensor.Encoder.vel.FRR;
    sharedData->joint_info[12+7] =      sensor.Encoder.vel.FRP;
    sharedData->joint_info[12+8] =      sensor.Encoder.vel.FRK;
    sharedData->joint_info[12+9] =      sensor.Encoder.vel.FLR;
    sharedData->joint_info[12+10] =     sensor.Encoder.vel.FLP;
    sharedData->joint_info[12+11] =     sensor.Encoder.vel.FLK;

    sharedData->joint_info[24+0] =      sensor.Encoder.acc.HRR;
    sharedData->joint_info[24+1] =      sensor.Encoder.acc.HRP;
    sharedData->joint_info[24+2] =      sensor.Encoder.acc.HRK;
    sharedData->joint_info[24+3] =      sensor.Encoder.acc.HLR;
    sharedData->joint_info[24+4] =      sensor.Encoder.acc.HLP;
    sharedData->joint_info[24+5] =      sensor.Encoder.acc.HLK;
    sharedData->joint_info[24+6] =      sensor.Encoder.acc.FRR;
    sharedData->joint_info[24+7] =      sensor.Encoder.acc.FRP;
    sharedData->joint_info[24+8] =      sensor.Encoder.acc.FRK;
    sharedData->joint_info[24+9] =      sensor.Encoder.acc.FLR;
    sharedData->joint_info[24+10] =     sensor.Encoder.acc.FLP;
    sharedData->joint_info[24+11] =     sensor.Encoder.acc.FLK;

    sharedData->joint_info[36+0] =      sensor.Encoder.current.HRR;
    sharedData->joint_info[36+1] =      sensor.Encoder.current.HRP;
    sharedData->joint_info[36+2] =      sensor.Encoder.current.HRK;
    sharedData->joint_info[36+3] =      sensor.Encoder.current.HLR;
    sharedData->joint_info[36+4] =      sensor.Encoder.current.HLP;
    sharedData->joint_info[36+5] =      sensor.Encoder.current.HLK;
    sharedData->joint_info[36+6] =      sensor.Encoder.current.FRR;
    sharedData->joint_info[36+7] =      sensor.Encoder.current.FRP;
    sharedData->joint_info[36+8] =      sensor.Encoder.current.FRK;
    sharedData->joint_info[36+9] =      sensor.Encoder.current.FLR;
    sharedData->joint_info[36+10] =     sensor.Encoder.current.FLP;
    sharedData->joint_info[36+11] =     sensor.Encoder.current.FLK;

    // filtered
    sharedData->joint_info_filtered[0] =        Quad.QuadSensor_filtered.Encoder.pos.HRR;
    sharedData->joint_info_filtered[1] =        Quad.QuadSensor_filtered.Encoder.pos.HRP;
    sharedData->joint_info_filtered[2] =        Quad.QuadSensor_filtered.Encoder.pos.HRK;
    sharedData->joint_info_filtered[3] =        Quad.QuadSensor_filtered.Encoder.pos.HLR;
    sharedData->joint_info_filtered[4] =        Quad.QuadSensor_filtered.Encoder.pos.HLP;
    sharedData->joint_info_filtered[5] =        Quad.QuadSensor_filtered.Encoder.pos.HLK;
    sharedData->joint_info_filtered[6] =        Quad.QuadSensor_filtered.Encoder.pos.FRR;
    sharedData->joint_info_filtered[7] =        Quad.QuadSensor_filtered.Encoder.pos.FRP;
    sharedData->joint_info_filtered[8] =        Quad.QuadSensor_filtered.Encoder.pos.FRK;
    sharedData->joint_info_filtered[9] =        Quad.QuadSensor_filtered.Encoder.pos.FLR;
    sharedData->joint_info_filtered[10] =       Quad.QuadSensor_filtered.Encoder.pos.FLP;
    sharedData->joint_info_filtered[11] =       Quad.QuadSensor_filtered.Encoder.pos.FLK;
    sharedData->joint_info_filtered[12+0] =     Quad.QuadSensor_filtered.Encoder.vel.HRR;
    sharedData->joint_info_filtered[12+1] =     Quad.QuadSensor_filtered.Encoder.vel.HRP;
    sharedData->joint_info_filtered[12+2] =     Quad.QuadSensor_filtered.Encoder.vel.HRK;
    sharedData->joint_info_filtered[12+3] =     Quad.QuadSensor_filtered.Encoder.vel.HLR;
    sharedData->joint_info_filtered[12+4] =     Quad.QuadSensor_filtered.Encoder.vel.HLP;
    sharedData->joint_info_filtered[12+5] =     Quad.QuadSensor_filtered.Encoder.vel.HLK;
    sharedData->joint_info_filtered[12+6] =     Quad.QuadSensor_filtered.Encoder.vel.FRR;
    sharedData->joint_info_filtered[12+7] =     Quad.QuadSensor_filtered.Encoder.vel.FRP;
    sharedData->joint_info_filtered[12+8] =     Quad.QuadSensor_filtered.Encoder.vel.FRK;
    sharedData->joint_info_filtered[12+9] =     Quad.QuadSensor_filtered.Encoder.vel.FLR;
    sharedData->joint_info_filtered[12+10] =    Quad.QuadSensor_filtered.Encoder.vel.FLP;
    sharedData->joint_info_filtered[12+11] =    Quad.QuadSensor_filtered.Encoder.vel.FLK;
    sharedData->joint_info_filtered[24+0] =     Quad.QuadSensor_filtered.Encoder.acc.HRR;
    sharedData->joint_info_filtered[24+1] =     Quad.QuadSensor_filtered.Encoder.acc.HRP;
    sharedData->joint_info_filtered[24+2] =     Quad.QuadSensor_filtered.Encoder.acc.HRK;
    sharedData->joint_info_filtered[24+3] =     Quad.QuadSensor_filtered.Encoder.acc.HLR;
    sharedData->joint_info_filtered[24+4] =     Quad.QuadSensor_filtered.Encoder.acc.HLP;
    sharedData->joint_info_filtered[24+5] =     Quad.QuadSensor_filtered.Encoder.acc.HLK;
    sharedData->joint_info_filtered[24+6] =     Quad.QuadSensor_filtered.Encoder.acc.FRR;
    sharedData->joint_info_filtered[24+7] =     Quad.QuadSensor_filtered.Encoder.acc.FRP;
    sharedData->joint_info_filtered[24+8] =     Quad.QuadSensor_filtered.Encoder.acc.FRK;
    sharedData->joint_info_filtered[24+9] =     Quad.QuadSensor_filtered.Encoder.acc.FLR;
    sharedData->joint_info_filtered[24+10] =    Quad.QuadSensor_filtered.Encoder.acc.FLP;
    sharedData->joint_info_filtered[24+11] =    Quad.QuadSensor_filtered.Encoder.acc.FLK;






}

void SetOneJointRef(double angle, int idx){

    // uiuk modified
//    if(MY_CONTROL_FB_GAIN[idx] < 1)
//    {
//        _DEV_MC[idx].MoveJoints.RefAngleCurrent = _DEV_MC[idx].CurrentPosition;
//        _DEV_MC[idx].MoveJoints.RefAngleToGo = _DEV_MC[idx].CurrentPosition;
//    }
//    else
//    {
//        _DEV_MC[idx].MoveJoints.RefAngleCurrent = angle;
//        _DEV_MC[idx].MoveJoints.RefAngleToGo = angle;
//    }

    _DEV_MC[idx].MoveJoints.RefAngleCurrent = angle;
    _DEV_MC[idx].MoveJoints.RefAngleToGo = angle;

}

void SetOneCurrentRef(double current, int idx){
    if(MY_CONTROL_FF_GAIN[idx] < 1)
    {
        _DEV_MC[idx].CurrentReference_mA = 0;
//        _DEV_MC[idx].CurrentReference_mA = current*1000*sign(_DEV_MC[idx].PPR);
    }
    else
    {
        _DEV_MC[idx].CurrentReference_mA = current*1000*sign(_DEV_MC[idx].PPR);
    }
}

QuadJoint vec2QJ(double vector[12]){
    QuadJoint QJ_temp;
    QJ_temp.HRR = vector[HRR];
    QJ_temp.HRP = vector[HRP];
    QJ_temp.HRK = vector[HRK];
    QJ_temp.HLR = vector[HLR];
    QJ_temp.HLP = vector[HLP];
    QJ_temp.HLK = vector[HLK];
    QJ_temp.FRR = vector[FRR];
    QJ_temp.FRP = vector[FRP];
    QJ_temp.FRK = vector[FRK];
    QJ_temp.FLR = vector[FLR];
    QJ_temp.FLP = vector[FLP];
    QJ_temp.FLK = vector[FLK];

    return QJ_temp;
}

double *QJ2vec(QuadJoint QJ){
    static double vec_temp[12];
    vec_temp[HRR] = QJ.HRR;
    vec_temp[HRP] = QJ.HRP;
    vec_temp[HRK] = QJ.HRK;
    vec_temp[HLR] = QJ.HLR;
    vec_temp[HLP] = QJ.HLP;
    vec_temp[HLK] = QJ.HLK;
    vec_temp[FRR] = QJ.FRR;
    vec_temp[FRP] = QJ.FRP;
    vec_temp[FRK] = QJ.FRK;
    vec_temp[FLR] = QJ.FLR;
    vec_temp[FLP] = QJ.FLP;
    vec_temp[FLK] = QJ.FLK;

    return vec_temp;
}

void SetAllJointRef(QuadJoint JointRef){
    double *Jref_temp = QJ2vec(JointRef);
    for(int i=0; i<MAX_MC; i++)
    {
        if(i%3 != 2)
            SetOneJointRef(Jref_temp[i] * R2Df, i);
        else
            SetOneJointRef((Jref_temp[i] - (Jref_temp[i-1] - h_hip*D2Rf)/11.0) * R2Df, i);
    }
}

void SetAllCurrentRef(QuadJoint Cref){
    double *Cref_temp = QJ2vec(Cref);
    for(int i=0; i<MAX_MC; i++)
    {
        SetOneCurrentRef(Cref_temp[i], i);
    }
}

double SetOneCurrent(double torque)
{
    double current = torque / Quad.torque_const;
    // Clipping
    current = current < Quad.current_limit_ ? current : Quad.current_limit_;
    current = current > -Quad.current_limit_ ? current : -Quad.current_limit_;
    return current;
}

QuadJoint Torque2Current(QuadJoint torque)
{
    QuadJoint current;
    current.HRR = SetOneCurrent(torque.HRR);
    current.HRP = SetOneCurrent(torque.HRP);
    current.HRK = SetOneCurrent(torque.HRK);
    current.HLR = SetOneCurrent(torque.HLR);
    current.HLP = SetOneCurrent(torque.HLP);
    current.HLK = SetOneCurrent(torque.HLK);
    current.FRR = SetOneCurrent(torque.FRR);
    current.FRP = SetOneCurrent(torque.FRP);
    current.FRK = SetOneCurrent(torque.FRK);
    current.FLR = SetOneCurrent(torque.FLR);
    current.FLP = SetOneCurrent(torque.FLP);
    current.FLK = SetOneCurrent(torque.FLK);
    return current;

}
