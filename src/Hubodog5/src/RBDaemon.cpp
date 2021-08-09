#include "RBDaemon.h"
#include <chrono>

//#include "RBMotion.h"
#include "main_controller.h"
#include "RBCommon.h"
#include "RBLANComm.h"
//#include "UIUK.h"

#ifdef SIMULATION
raisim::RaisimServer *server;
raisim::ArticulatedSystem *robot;
#endif

pRBCORE_SHM sharedData;
RT_TASK     rtTaskCon;
RT_TASK     rtTaskCon2;


// Devices --------
RBMotorController   _DEV_MC[MAX_MC];
RBIMUSensor         _DEV_IMU;

extern RBLANComm   *lanHandler;




extern int     _IS_CAN_OK;
extern int     _NO_OF_MC;
extern int     _IS_WORKING;




bool StatusReadFlag[MAX_MC] = {0,};
long _ThreadCnt = 0;
bool ErrorClearStart = false;

using namespace std;

// Initialize Functions -------------------------------
void RBCore_SMInitialize(){



    sharedData = (pRBCORE_SHM)malloc(sizeof(RBCORE_SHM));
    sharedData->qPel[0] = 1;
    sharedData->qPel[1] = 0;
    sharedData->qPel[2] = 0;
    sharedData->qPel[3] = 0;
    for(int i=0; i<_NO_OF_MC; i++){
        for(int j=0; j<_DEV_MC[i].MOTOR_CHANNEL; j++){
            _DEV_MC[i].EncoderValue = 0;
            _DEV_MC[i].CurrentPosition = 0;
        //    sharedData->JointReference[i][j] = 0.0;
          //  _DEV_MC[i].Reference = 0.0;
//            _DEV_MC[i].MoveJoints.RefAngleCurrent = 0.0;
//            _DEV_MC[i].RB_InitControl();
        }
    }
    sharedData->COMMAND.USER_COMMAND = 0;
    sharedData->NEWCOMMAND = false;

    // custom SMs

    for(int i=0; i<3; i++)
    {
        sharedData->Kp[i] = 0;
        sharedData->Kd[i] = 0;
    }
    for(int i=0; i<4; i++)
    {
        sharedData->idx_legcon[i] = false;
    }
    for(int i=0; i<4; i++)
    {
        sharedData->demo_variable[i] = 0;
    }
    for(int i=0; i<12; i++)
    {
        sharedData->pf_ref[i] = 0;
        sharedData->pf_real[i] = 0;
        sharedData->PCON[i] = 0;
        sharedData->CCON[i] = 0;
        sharedData->joint_IK[i] = 0;
    }
    for(int i=0; i<36; i++)
    {
        sharedData->joint_info[i] = 0;
        sharedData->joint_info_filtered[i] = 0;
        sharedData->joint_info_ref[i] = 0;
    }
    sharedData->isSimulation = false;
    sharedData->isIMUAlwaysZero = false;
//    sharedData->plot_play_ON = false;
    sharedData->plot_timer_zero  = false;
    sharedData->z0 = 0;
    sharedData->x0 = 0;
    sharedData->Cmax = 0;
    sharedData->isJointFilter = false;
    sharedData->frq_JointFilter = 500;
    sharedData->btn_IMUFilter = true;

    for(int i=0; i<12; i++)
    {
        sharedData->PCON[i]=0;
        sharedData->CCON[i]=0;
    }
    sharedData->plot_play_ON = true;
    sharedData->plot_timer_zero = false;
}


int RBCore_ThreadInitialize(){
    if(rt_task_create(&rtTaskCon, "RCR_CORE", 0, 99, 0) == 0){
        cpu_set_t aCPU;
        CPU_ZERO(&aCPU);
        CPU_SET(0, &aCPU);
        if(rt_task_set_affinity(&rtTaskCon, &aCPU) != 0){
//            cout << "[ERR] Core real-time thread set affinity CPU failed.." << endl;
//            RCR_LOG(LOG_TYPE_FATAL, "RT AFFINITY FAIL");
        }
        if(rt_task_start(&rtTaskCon, &RBCore_RTThreadCon, NULL) == 0){
//            cout << "[OK] Core real-time thread start = OK" << endl;
//            RCR_LOG(LOG_TYPE_SYSTEM, "RT INIT OK");
        }else{
//            cout << "[ERR] Core real-time thread start = FAIL" << endl;
//            RCR_LOG(LOG_TYPE_FATAL, "RT INIT FAIL");
            return false;
        }
    }else{
//        cout << "[ERR] Fail to create core real-time thread" << endl;
//        RCR_LOG(LOG_TYPE_FATAL, "RT INIT FAIL");
        return false;
    }
    if(rt_task_create(&rtTaskCon2, "RCR_CORE2", 0, 99, 0) == 0){
        cpu_set_t aCPU;
        CPU_ZERO(&aCPU);
        CPU_SET(2, &aCPU);
        if(rt_task_set_affinity(&rtTaskCon2, &aCPU) != 0){
//            cout << "[ERR] Core real-time thread set affinity CPU failed.." << endl;
//            RCR_LOG(LOG_TYPE_FATAL, "RT AFFINITY FAIL");
        }
        if(rt_task_start(&rtTaskCon2, &RBCore_RTThreadCon2, NULL) == 0){
//            cout << "[OK] Core real-time thread start = OK" << endl;
//            RCR_LOG(LOG_TYPE_SYSTEM, "RT INIT OK");
        }else{
//            cout << "[ERR] Core real-time thread start = FAIL" << endl;
//            RCR_LOG(LOG_TYPE_FATAL, "RT INIT FAIL");
            return false;
        }
    }else{
//        cout << "[ERR] Fail to create core real-time thread" << endl;
//        RCR_LOG(LOG_TYPE_FATAL, "RT INIT FAIL");
        return false;
    }
    return true;
}



int RBCore_CANInitialize(){
    canHandler = new RBCAN(1);

    if(canHandler->IsWorking() == false){
        _IS_CAN_OK = false;
        return false;
    }else{
        for(int i=0; i<_NO_OF_MC; i++)
            _DEV_MC[i].RBMC_AddCANMailBox();
        _IS_CAN_OK = true;
        return true;
    }
}




void *RBDaemon_Network(void *){

    printf("Starting RBDaemon_Network Thread\n");
    while(_IS_WORKING)
    {
        lanHandler->RBLANComm_ReadData();
    #ifdef SIMULATION

    #endif
        usleep(10);
    }
    return NULL;
}

double delX, delY, delZ;
double tempX, tempY, tempZ;

void RBCore_RTThreadCon2(void *)
{
    Eigen::Vector3d angle, gyro, acc;
#ifdef SIMULATION
  raisim::Vec<3> imu_vel, imu_vel_prev, imu_ang_vel;
  Eigen::Matrix3d rotation;
  imu_vel_prev.setZero();

  Eigen::VectorXd gc(19), gv(18);
  robot->getState(gc, gv);
  for (int i = 0; i < 12; i++)
    _DEV_MC[i].MoveJoints.RefAngleCurrent = R2D*(gc(7 + i));
#endif

    while(_IS_WORKING)
    {
        //thread_MPC();
        rt_task_suspend(&rtTaskCon2);

#ifdef SIMULATION
        if(sharedData->RaisimSimulation == true) {
            // Generate IMU data from Raisim
            robot->getFrameVelocity("imu_joint", imu_vel);
            robot->getFrameAngularVelocity("imu_joint", imu_ang_vel);
            rotation = robot->getBaseOrientation().e();

            acc = (imu_vel.e() - imu_vel_prev.e()) * 100; // 100 is estimation frequency
            acc += Eigen::Vector3d(0, 0, 9.81); // Gravity
            acc = rotation.transpose() * acc; // From world to Body frmae
            imu_vel_prev = imu_vel;

            sharedData->IMU[0].AccX = acc[0];
            sharedData->IMU[0].AccY = acc[1];
            sharedData->IMU[0].AccZ = acc[2];

            gyro = rotation.transpose() * imu_ang_vel.e();

            double velX = gyro.x();
            double velY = gyro.y();
            double velZ = gyro.z();

            delX = velX + velY*sin(tempX)*tan(tempY) + velZ*cos(tempX)*tan(tempY);
            delY = velY*cos(tempX) - velZ*sin(tempX);
            delZ = velY*sin(tempX)/(cos(tempY)) + velZ*cos(tempX)/(cos(tempY)); // it does not return

            tempX += delX*0.002;
            tempY += delY*0.002;
            tempZ += delZ*0.002;

//            tempX += velX*0.002;
//            tempY += velY*0.002;
//            tempZ += velZ*0.002;

            angle[0] = tempX;
            angle[1] = tempY;
            angle[2] = tempZ;

            sharedData->IMU[0].RollVel = gyro[0]*R2D;
            sharedData->IMU[0].PitchVel = gyro[1]*R2D;
            sharedData->IMU[0].YawVel = gyro[2]*R2D;

            sharedData->IMU[0].Roll = angle[0]*R2D;
            sharedData->IMU[0].Pitch = angle[1]*R2D;
            sharedData->IMU[0].Yaw = angle[2]*R2D;

            robot->getState(gc, gv);
            Eigen::VectorXd torques(18);

            for(int i=0; i<18; i++)
                torques[i] = robot->getGeneralizedForce()[i];

            for(int i=0; i<12; i++) {
                _DEV_MC[i].CurrentPosition = gc(7+i)*R2D;
                _DEV_MC[i].CurrentVelocity = gc(6+i)*R2D;
                _DEV_MC[i].MeasuredCurrent = torques[6+i] / 0.125 / 11.0 * 1000.0;
            }

            //FILE_LOG(logDEBUG) << "hi";

            //quad.setJoints(gc.tail(12), gv.tail(12));
        }
        #endif
    }
}

int MY_CONTROL_FB_GAIN[12] = {10,10,10,10,10,10,10,10,10,10,10,10}; // default gain : 10/10
int MY_CONTROL_FF_GAIN[12] = {10,10,10,10,10,10,10,10,10,10,10,10}; // default gain : 10/10
int  enc_request_time_array[MAX_MC] = {
    0,0,0,
    0,0,0,
    0,0,0,
    0,0,0
};
int init_state = 0;
int delay_cnt = 0;

void RBCore_RTThreadCon(void *)
{
    static int enc_toggle = 0;

//    rt_task_set_periodic(NULL, TM_NOW, (RT_TIMER_PERIOD_MS)*1000000);
    rt_task_set_periodic(NULL, TM_NOW, (RT_TIMER_PERIOD_MS)*1000000);

    pthread_t hThread;
    pthread_create_with_affinity(RBDaemon_Network,1,"RBDaemonNetwork",hThread,NULL);
    main_init();


    while(_IS_WORKING){

        rt_task_wait_period(NULL);

        canHandler->RBCAN_SendData();

        static int enc_request_flag = 0;
        if(init_state == 1) {
            if(enc_request_flag == 0) {
                for(int i=0; i<_NO_OF_MC; i++){
                    _DEV_MC[i].RBBoard_RequestEncoder(0);
                }
                enc_request_flag = 1;
                cout << "request encoder" << endl;
            }
        }


        rt_task_resume(&rtTaskCon2);


//        set_ref_data();
//        get_sensor_data();
        main_controller();

        //cout << "pos before ref : " << _DEV_MC[1].MoveJoints.RefAngleCurrent << endl;

        for(int i=0; i<_NO_OF_MC; i++){
            if(MY_CONTROL_FB_GAIN[i]>0){
                _DEV_MC[i].RBJoint_MoveJoint();
            }

            //sharedData->JointReference[i][0] = _DEV_MC[i].MoveJoints.RefAngleCurrent - _DEV_MC[i].homeJointOffset;
            sharedData->JointReference[i][0] = _DEV_MC[i].MoveJoints.RefAngleCurrent - _DEV_MC[i].homeJointOffset - _DEV_MC[i].homeJointOffset_mult;
            _DEV_MC[i].Reference = sharedData->JointReference[i][0];



            if(lanHandler->ConnectionStatus == true && _DEV_MC[i].ConnectionStatus == true){
                _DEV_MC[i].RBBoard_SendReference(enc_request_time_array[i]);
            }
        }

#ifdef SIMULATION
    sharedData->RaisimSimulation = true;
#else
    sharedData->RaisimSimulation = false;
#endif


    if(sharedData->RaisimResetFlag == true) {
        sharedData->RaisimResetFlag = false;
#ifdef SIMULATION
        Eigen::VectorXd joint_pos(robot->getGeneralizedCoordinateDim());
        joint_pos.setZero();
        for (int i = 0; i < 4; i++)
          joint_pos.segment(7 + 3*i, 3) = Eigen::Vector3d(0, 30*D2R, -2 * 30*D2R);
        joint_pos(3) = 1;
        joint_pos(2) = 0.45;


        robot->setGeneralizedCoordinate(joint_pos);
#endif
    }

#ifdef SIMULATION

//        robot->setControlMode(raisim::ControlMode::PD_PLUS_FEEDFORWARD_TORQUE);
//        Eigen::VectorXd jointPgain(robot->getDOF()), jointDgain(robot->getDOF());
//        jointPgain.tail(12).setConstant(400);
//        jointDgain.tail(12).setConstant(10);
//        robot->setPdGains(jointPgain, jointDgain);
        //

    Eigen::VectorXd torques(18);
    Eigen::VectorXd joint_ref;
    Eigen::VectorXd jointPgain(robot->getDOF()), jointDgain(robot->getDOF());

    auto gc = robot->getGeneralizedCoordinate();
    joint_ref.setZero(19);
    joint_ref.head(7) = gc.e().head(7);

    robot->setControlMode(raisim::ControlMode::PD_PLUS_FEEDFORWARD_TORQUE);

//    if(sharedData->RaisimGainState == true) {
//        // control on - torque control
//        for(int i=0; i<12; i++) {
//            torques(6+i) = _DEV_MC[i].CurrentReference_mA/1000*_DEV_MC[i].TPC;
//        }
//        robot->setGeneralizedForce(torques);

//        jointPgain.tail(12).setConstant(0);
//        jointDgain.tail(12).setConstant(5);
//    }
//    else {
//        // control off - position control
//        jointPgain.tail(12).setConstant(400);
//        jointDgain.tail(12).setConstant(5);
//    }

    jointPgain.tail(12).setConstant(500);
    jointDgain.tail(12).setConstant(1);

    raisim::Vec<2> joint_limit = {-3600*D2R, 3600*D2R};
    std::vector<raisim::Vec<2>> joints_limit(robot->getDOF());


    //robot->setJointLimits();

    robot->setPdGains(jointPgain, jointDgain);
//    joint_ref(3) = 1;
//    joint_ref(2) = 1;

    for (int i = 0; i < 12; i++){
        if(i == HRK || i == HLK || i == FRK || i == FLK){
            joint_ref(7 + i) = D2R*(_DEV_MC[i].MoveJoints.RefAngleCurrent + _DEV_MC[i-1].MoveJoints.RefAngleCurrent/11.0);
        }
        joint_ref(7 + i) = D2R*(_DEV_MC[i].MoveJoints.RefAngleCurrent);
    }


    //robot->setGeneralizedCoordinate(joint_ref);
    robot->setPTarget(joint_ref);


    server->integrateWorldThreadSafe();
#else

#endif

    }
}

