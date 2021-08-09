#include "RBMotion.h"
#include "JointInformation.h"
#include "RBSharedMemory.h"

//#include <alchemy/task.h>
#include "pthread.h"


#include "RBMotorController.h"
#include "RBCAN.h"
#include <sched.h>
#include <sys/types.h>

#include <libgen.h>

#include <stdio.h>
#include <dirent.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <stdarg.h>


#include <deque>
#include "rbdl/rbdl.h"


using namespace std;


extern RBMotorController   _DEV_MC[MAX_MC];

using namespace std;
bool do_falldownmotion = false;
int falldownmotionCnt = 0;
int RBCore_DBInitialize()
{
    _NO_OF_MC = 12;
    _DEV_MC[0].RBBoard_SetCANIDs(0, 1, 0);     // J0
    _DEV_MC[1].RBBoard_SetCANIDs(1, 1, 0);     // J1
    _DEV_MC[2].RBBoard_SetCANIDs(2, 1, 0);     // J2
    _DEV_MC[3].RBBoard_SetCANIDs(3, 1, 0);     // J3
    _DEV_MC[4].RBBoard_SetCANIDs(4, 1, 0);     // J4
    _DEV_MC[5].RBBoard_SetCANIDs(5, 1, 0);     // J5
    _DEV_MC[6].RBBoard_SetCANIDs(6, 1, 1);     // J6
    _DEV_MC[7].RBBoard_SetCANIDs(7, 1, 1);     // J7
    _DEV_MC[8].RBBoard_SetCANIDs(8, 1, 1);     // J8
    _DEV_MC[9].RBBoard_SetCANIDs(9, 1, 1);     // J9
    _DEV_MC[10].RBBoard_SetCANIDs(10, 1, 1);     // J10
    _DEV_MC[11].RBBoard_SetCANIDs(11, 1, 1);     // J11


    _DEV_MC[0].TPC =   1;
    _DEV_MC[1].TPC =  -1;
    _DEV_MC[2].TPC =   1;
    _DEV_MC[3].TPC =  -1;
    _DEV_MC[4].TPC =  -1;
    _DEV_MC[5].TPC =   1;
    _DEV_MC[6].TPC =   1;
    _DEV_MC[7].TPC =  -1;
    _DEV_MC[8].TPC =  -1;
    _DEV_MC[9].TPC =   1;
    _DEV_MC[10].TPC =  1;
    _DEV_MC[11].TPC =  1;

    double ratio = 11.0;
    for(int i=0; i<12; i++)
        _DEV_MC[i].PPR = _DEV_MC[i].TPC*16384.0*ratio/360.0;     // HR_HR

    _DEV_MC[0].homeJointLift  = 17.0;
    _DEV_MC[1].homeJointLift  = 10.0;
    _DEV_MC[2].homeJointLift  = -30.0 -10/11.0;
    _DEV_MC[3].homeJointLift  = -17.0;
    _DEV_MC[4].homeJointLift  = 10.0;
    _DEV_MC[5].homeJointLift  = -30.0 -10/11.0;
    _DEV_MC[6].homeJointLift  = 17.0;
    _DEV_MC[7].homeJointLift  = 10.0;
    _DEV_MC[8].homeJointLift  = -30.0 -10/11.0;
    _DEV_MC[9].homeJointLift  = -17.0;
    _DEV_MC[10].homeJointLift = 10.0;
    _DEV_MC[11].homeJointLift = -30.0 -10/11.0;

    _DEV_MC[0].homeJointReady = 0;
    _DEV_MC[1].homeJointReady = 35.0;
    _DEV_MC[2].homeJointReady = -70.0 -35/11.0;
    _DEV_MC[3].homeJointReady = 0;
    _DEV_MC[4].homeJointReady = 35.0;
    _DEV_MC[5].homeJointReady = -70.0 -35/11.0;
    _DEV_MC[6].homeJointReady = 0;
    _DEV_MC[7].homeJointReady = 35.0;
    _DEV_MC[8].homeJointReady = -70.0 -35/11.0;
    _DEV_MC[9].homeJointReady = 0;
   _DEV_MC[10].homeJointReady = 35.0;
   _DEV_MC[11].homeJointReady = -70.0 -35/11.0;

//   _DEV_MC[0].homeJointReady = 0;
//   _DEV_MC[1].homeJointReady = 30.0;
//   _DEV_MC[2].homeJointReady = -60.0 -30/11.0;
//   _DEV_MC[3].homeJointReady = 0;
//   _DEV_MC[4].homeJointReady = 30.0;
//   _DEV_MC[5].homeJointReady = -60.0 -30/11.0;
//   _DEV_MC[6].homeJointReady = 0;
//   _DEV_MC[7].homeJointReady = 30.0;
//   _DEV_MC[8].homeJointReady = -60.0 -30/11.0;
//   _DEV_MC[9].homeJointReady = 0;
//  _DEV_MC[10].homeJointReady = 30.0;
//  _DEV_MC[11].homeJointReady = -60.0 -30/11.0;

   _DEV_MC[0].homeJointGround = 0;
   _DEV_MC[1].homeJointGround = 150.0;
   _DEV_MC[2].homeJointGround = -150.0 - 150/11.0;
   _DEV_MC[3].homeJointGround = 0;
   _DEV_MC[4].homeJointGround = 150.0;
   _DEV_MC[5].homeJointGround = -150.0 - 150/11.0;
   _DEV_MC[6].homeJointGround = 0;
   _DEV_MC[7].homeJointGround = 150.0;
   _DEV_MC[8].homeJointGround = -150.0 - 150/11.0;
   _DEV_MC[9].homeJointGround = 0;
  _DEV_MC[10].homeJointGround = 150.0;
  _DEV_MC[11].homeJointGround = -150.0 - 150/11.0;

  double kn_offset = 9.83;
  _DEV_MC[0].homeJointOffset  =  0;
  _DEV_MC[1].homeJointOffset  =  180.0;
  _DEV_MC[2].homeJointOffset  = (-90.0+kn_offset) - 180/11.0;
  _DEV_MC[3].homeJointOffset  =  0;
  _DEV_MC[4].homeJointOffset  =  180.0;
  _DEV_MC[5].homeJointOffset  = (-90.0+kn_offset) - 180/11.0;
  _DEV_MC[6].homeJointOffset  =  0;
  _DEV_MC[7].homeJointOffset  =  180.0;
  _DEV_MC[8].homeJointOffset  = (-90.0+kn_offset) - 180/11.0;
  _DEV_MC[9].homeJointOffset  =  0;
  _DEV_MC[10].homeJointOffset =  180.0;
  _DEV_MC[11].homeJointOffset = (-90.0+kn_offset) - 180/11.0;

//  _DEV_MC[0].homeJointOffset  = 0;    // HR_HR
//  _DEV_MC[1].homeJointOffset  = 180.0                 -2;   // HR_HP
//  _DEV_MC[2].homeJointOffset  = -142.0 - 150/11;   // HR_KN
//  _DEV_MC[3].homeJointOffset  = 0;    // HL_HR
//  _DEV_MC[4].homeJointOffset  = 180.0;   // HL_HP
//  _DEV_MC[5].homeJointOffset  = -142.0-3 - 150/11;     // HL_KN
//  _DEV_MC[6].homeJointOffset  = 0                     -5;     // FR_HR
//  _DEV_MC[7].homeJointOffset  = 180.0                 -2;   // FR_HP
//  _DEV_MC[8].homeJointOffset  = -142.0-5 - 150/11;     // FR_KN
//  _DEV_MC[9].homeJointOffset  =  0;   // FL_HR
//  _DEV_MC[10].homeJointOffset =  180.0;   // FL_HP
//  _DEV_MC[11].homeJointOffset = -142.0 - 150/11;     // FL_KN

    _DEV_MC[0].CAN_CHANNEL = 0;
    _DEV_MC[1].CAN_CHANNEL = 0;
    _DEV_MC[2].CAN_CHANNEL = 0;
    _DEV_MC[3].CAN_CHANNEL = 0;
    _DEV_MC[4].CAN_CHANNEL = 0;
    _DEV_MC[5].CAN_CHANNEL = 0;
    _DEV_MC[6].CAN_CHANNEL = 1;
    _DEV_MC[7].CAN_CHANNEL = 1;
    _DEV_MC[8].CAN_CHANNEL = 1;
    _DEV_MC[9].CAN_CHANNEL = 1;
    _DEV_MC[10].CAN_CHANNEL = 1;
    _DEV_MC[11].CAN_CHANNEL = 1;

    return true;
}

/*****************************************************************************************************************
 * SYS Functions
 *************************************************************************************************************** */
int SYS_SET_THREAD_AFFINITY(void* (*t_handler)(void *), int t_cpu_no, const char* t_name)
{
    pthread_t thread_nrt;
    pthread_attr_t attr;
    cpu_set_t cpuset;
    int err;

    // initialized with default attributes
    err = pthread_attr_init(&attr);
    if (err != 0) {
//        cout<<"[ERR] pthread_attr_init"<<endl;
//        RCR_LOG(LOG_TYPE_FATAL, "THREAD INIT FAIL");
        return -1;
    }
    // set cpu ID
    if (t_cpu_no >= 0) {
        CPU_ZERO(&cpuset);
        CPU_SET(t_cpu_no, &cpuset);
        err = pthread_attr_setaffinity_np(&attr, sizeof(cpu_set_t), &cpuset);
        if (err != 0){
//            cout<<"[ERR] pthread_attr_setaffinity_np"<<endl;
//            RCR_LOG(LOG_TYPE_FATAL, "THREAD AFFINITY FAIL");
        }
    }
    // create a RT thread
    err = pthread_create(&thread_nrt, &attr, t_handler, NULL);
    if (err != 0) {
//        cout<<"[ERR] pthread_create(TimerThread)"<<endl;
//        RCR_LOG(LOG_TYPE_FATAL, "THREAD CREATE FAIL");
        err = pthread_attr_destroy(&attr);
        if (err != 0){
//            cout<<"[ERR] pthread_attr_destroy"<<endl;
//            RCR_LOG(LOG_TYPE_FATAL, "THREAD DESTROY FAIL");
        }
        return -1;
    }

    pthread_setname_np(thread_nrt, t_name);


    return 1;
}


