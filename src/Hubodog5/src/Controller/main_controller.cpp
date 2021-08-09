// main controller
#include "main_controller.h"
#include "RBMotion.h"
#include "RBSharedMemory.h"
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

#include "hubodog5_general.h"
#include "func_sensor.h"

extern RBMotorController   _DEV_MC[MAX_MC];
extern RBCAN    *canHandler;
extern pRBCORE_SHM sharedData;

extern int MY_CONTROL_FB_GAIN[12];
extern int MY_CONTROL_FF_GAIN[12];

hubodog5_general Quad;
SensorInfo sensor, sensor_ref;

Vector3d angle_ready = Vector3d(0, 30, -60);
Vector3d angle_mount = Vector3d(0, 20, -40);
Vector3d angle_ground = Vector3d(0, 150, -150);
Vector3d angle_ground2 = Vector3d(0, 70, -150);
int ready_ms = 1000;

int Joint_mov_flag = 0;
double ref_angle[2]      = {0,};
double angle_err[2]      = {0,};
double angle_err_old[2]  = {0,};
double angle_err_dif[2]  = {0,};
double ref_current[2]    = {0,};
double current_limit[2]  = {0,};
double angle_mov[2]      = {0,};
double angle_rel[12] = {0., };

int REF_modi_cnt = 0;
bool incgain2_flag = false;
double QJ_ref[12];

//QuadJoint Cref;
//QuadJoint JointRef;
void main_init(){
//    FILE_LOG(logSUCCESS) << "Initialize Main Controller";
    SYS_SET_THREAD_AFFINITY(&main_nonRT,7,"command");
}

void *main_nonRT(void *arg){
    while(_IS_WORKING)
    {
//       cout << "NonRT" << endl;
        if(sharedData->NEWCOMMAND)
        {
//            FILE_LOG(logSUCCESS) << "COMMAND DETECTED.";
            sharedData->NEWCOMMAND = false;
            int idx = sharedData->COMMAND.USER_COMMAND;
            cout << "idx : " << idx << endl;
            switch(idx)
            {
            case READYPOS:
            {
                sharedData->COMMAND.USER_COMMAND = NO_ACT;
                if(sharedData->isStopping == false) {
                    FILE_LOG(logSUCCESS) << "HUBODOG5_READYPOS";
                    for(int i=0;i<NO_OF_JOINTS;i++)
                    {
                        MY_CONTROL_FB_GAIN[i] = 10;
                        MY_CONTROL_FF_GAIN[i] = 0;
                    }

                    QuadJoint QJ;

                    if(_DEV_MC[HRP].MoveJoints.RefAngleCurrent > 90) {
                        FILE_LOG(logSUCCESS) << "GROUND to READY";

                        QJ.HRR = angle_ground2[0];
                        QJ.HRP = angle_ground2[1];
                        QJ.HRK = angle_ground2[2];
                        QJ.HLR = angle_ground2[0];
                        QJ.HLP = angle_ground2[1];
                        QJ.HLK = angle_ground2[2];
                        QJ.FRR = angle_ground2[0];
                        QJ.FRP = angle_ground2[1];
                        QJ.FRK = angle_ground2[2];
                        QJ.FLR = angle_ground2[0];
                        QJ.FLP = angle_ground2[1];
                        QJ.FLK = angle_ground2[2];

                        _DEV_MC[HRR].RBJoint_SetMoveJoint(QJ.HRR, ready_ms, MOVE_ABSOLUTE);
                        _DEV_MC[HRP].RBJoint_SetMoveJoint(QJ.HRP, ready_ms, MOVE_ABSOLUTE);
                        _DEV_MC[HRK].RBJoint_SetMoveJoint(QJ.HRK, ready_ms, MOVE_ABSOLUTE);
                        _DEV_MC[HLR].RBJoint_SetMoveJoint(QJ.HLR, ready_ms, MOVE_ABSOLUTE);
                        _DEV_MC[HLP].RBJoint_SetMoveJoint(QJ.HLP, ready_ms, MOVE_ABSOLUTE);
                        _DEV_MC[HLK].RBJoint_SetMoveJoint(QJ.HLK, ready_ms, MOVE_ABSOLUTE);
                        _DEV_MC[FRR].RBJoint_SetMoveJoint(QJ.FRR, ready_ms, MOVE_ABSOLUTE);
                        _DEV_MC[FRP].RBJoint_SetMoveJoint(QJ.FRP, ready_ms, MOVE_ABSOLUTE);
                        _DEV_MC[FRK].RBJoint_SetMoveJoint(QJ.FRK, ready_ms, MOVE_ABSOLUTE);
                        _DEV_MC[FLR].RBJoint_SetMoveJoint(QJ.FLR, ready_ms, MOVE_ABSOLUTE);
                        _DEV_MC[FLP].RBJoint_SetMoveJoint(QJ.FLP, ready_ms, MOVE_ABSOLUTE);
                        _DEV_MC[FLK].RBJoint_SetMoveJoint(QJ.FLK, ready_ms, MOVE_ABSOLUTE);

                        usleep(ready_ms * 1e3);

                        usleep(100 * 1e3);

                        QJ.HRR = _DEV_MC[HRR].homeJointReady;
                        QJ.HRP = _DEV_MC[HRP].homeJointReady;
                        QJ.HRK = _DEV_MC[HRK].homeJointReady;
                        QJ.HLR = _DEV_MC[HLR].homeJointReady;
                        QJ.HLP = _DEV_MC[HLP].homeJointReady;
                        QJ.HLK = _DEV_MC[HLK].homeJointReady;
                        QJ.FRR = _DEV_MC[FRR].homeJointReady;
                        QJ.FRP = _DEV_MC[FRP].homeJointReady;
                        QJ.FRK = _DEV_MC[FRK].homeJointReady;
                        QJ.FLR = _DEV_MC[FLR].homeJointReady;
                        QJ.FLP = _DEV_MC[FLP].homeJointReady;
                        QJ.FLK = _DEV_MC[FLK].homeJointReady;

                        _DEV_MC[HRR].RBJoint_SetMoveJoint(QJ.HRR, ready_ms, MOVE_ABSOLUTE);
                        _DEV_MC[HRP].RBJoint_SetMoveJoint(QJ.HRP, ready_ms, MOVE_ABSOLUTE);
                        _DEV_MC[HRK].RBJoint_SetMoveJoint(QJ.HRK, ready_ms, MOVE_ABSOLUTE);
                        _DEV_MC[HLR].RBJoint_SetMoveJoint(QJ.HLR, ready_ms, MOVE_ABSOLUTE);
                        _DEV_MC[HLP].RBJoint_SetMoveJoint(QJ.HLP, ready_ms, MOVE_ABSOLUTE);
                        _DEV_MC[HLK].RBJoint_SetMoveJoint(QJ.HLK, ready_ms, MOVE_ABSOLUTE);
                        _DEV_MC[FRR].RBJoint_SetMoveJoint(QJ.FRR, ready_ms, MOVE_ABSOLUTE);
                        _DEV_MC[FRP].RBJoint_SetMoveJoint(QJ.FRP, ready_ms, MOVE_ABSOLUTE);
                        _DEV_MC[FRK].RBJoint_SetMoveJoint(QJ.FRK, ready_ms, MOVE_ABSOLUTE);
                        _DEV_MC[FLR].RBJoint_SetMoveJoint(QJ.FLR, ready_ms, MOVE_ABSOLUTE);
                        _DEV_MC[FLP].RBJoint_SetMoveJoint(QJ.FLP, ready_ms, MOVE_ABSOLUTE);
                        _DEV_MC[FLK].RBJoint_SetMoveJoint(QJ.FLK, ready_ms, MOVE_ABSOLUTE);

                        usleep(ready_ms * 1e3);
                    }
                    else {
                        FILE_LOG(logSUCCESS) << "STAND to READY";

                        QJ.HRR = _DEV_MC[HRR].homeJointReady;
                        QJ.HRP = _DEV_MC[HRP].homeJointReady;
                        QJ.HRK = _DEV_MC[HRK].homeJointReady;
                        QJ.HLR = _DEV_MC[HLR].homeJointReady;
                        QJ.HLP = _DEV_MC[HLP].homeJointReady;
                        QJ.HLK = _DEV_MC[HLK].homeJointReady;
                        QJ.FRR = _DEV_MC[FRR].homeJointReady;
                        QJ.FRP = _DEV_MC[FRP].homeJointReady;
                        QJ.FRK = _DEV_MC[FRK].homeJointReady;
                        QJ.FLR = _DEV_MC[FLR].homeJointReady;
                        QJ.FLP = _DEV_MC[FLP].homeJointReady;
                        QJ.FLK = _DEV_MC[FLK].homeJointReady;

                        _DEV_MC[HRR].RBJoint_SetMoveJoint(QJ.HRR, ready_ms, MOVE_ABSOLUTE);
                        _DEV_MC[HRP].RBJoint_SetMoveJoint(QJ.HRP, ready_ms, MOVE_ABSOLUTE);
                        _DEV_MC[HRK].RBJoint_SetMoveJoint(QJ.HRK, ready_ms, MOVE_ABSOLUTE);
                        _DEV_MC[HLR].RBJoint_SetMoveJoint(QJ.HLR, ready_ms, MOVE_ABSOLUTE);
                        _DEV_MC[HLP].RBJoint_SetMoveJoint(QJ.HLP, ready_ms, MOVE_ABSOLUTE);
                        _DEV_MC[HLK].RBJoint_SetMoveJoint(QJ.HLK, ready_ms, MOVE_ABSOLUTE);
                        _DEV_MC[FRR].RBJoint_SetMoveJoint(QJ.FRR, ready_ms, MOVE_ABSOLUTE);
                        _DEV_MC[FRP].RBJoint_SetMoveJoint(QJ.FRP, ready_ms, MOVE_ABSOLUTE);
                        _DEV_MC[FRK].RBJoint_SetMoveJoint(QJ.FRK, ready_ms, MOVE_ABSOLUTE);
                        _DEV_MC[FLR].RBJoint_SetMoveJoint(QJ.FLR, ready_ms, MOVE_ABSOLUTE);
                        _DEV_MC[FLP].RBJoint_SetMoveJoint(QJ.FLP, ready_ms, MOVE_ABSOLUTE);
                        _DEV_MC[FLK].RBJoint_SetMoveJoint(QJ.FLK, ready_ms, MOVE_ABSOLUTE);

                        usleep(ready_ms * 1e3);
                    }

                    sharedData->isFalled = false;
                    FILE_LOG(logSUCCESS) << "READYPOS Done.";
                    Quad.isReadyDone = true;

                }


                break;
            }
            case MOUNTPOS:
            {
                sharedData->COMMAND.USER_COMMAND = NO_ACT;
                FILE_LOG(logSUCCESS) << "HUBODOG5_MOUNTPOS";
                for(int i=0;i<NO_OF_JOINTS;i++)
                {
                    MY_CONTROL_FB_GAIN[i] = 10;
                    MY_CONTROL_FF_GAIN[i] = 0;
                }

                QuadJoint QJ;
                QJ.HRR = angle_mount[0];
                QJ.HRP = angle_mount[1];
                QJ.HRK = angle_mount[2];
                QJ.HLR = angle_mount[0];
                QJ.HLP = angle_mount[1];
                QJ.HLK = angle_mount[2];
                QJ.FRR = angle_mount[0];
                QJ.FRP = angle_mount[1];
                QJ.FRK = angle_mount[2];
                QJ.FLR = angle_mount[0];
                QJ.FLP = angle_mount[1];
                QJ.FLK = angle_mount[2];

                _DEV_MC[HRR].RBJoint_SetMoveJoint(QJ.HRR, ready_ms, MOVE_ABSOLUTE);
                _DEV_MC[HRP].RBJoint_SetMoveJoint(QJ.HRP, ready_ms, MOVE_ABSOLUTE);
                _DEV_MC[HRK].RBJoint_SetMoveJoint(QJ.HRK, ready_ms, MOVE_ABSOLUTE);
                _DEV_MC[HLR].RBJoint_SetMoveJoint(QJ.HLR, ready_ms, MOVE_ABSOLUTE);
                _DEV_MC[HLP].RBJoint_SetMoveJoint(QJ.HLP, ready_ms, MOVE_ABSOLUTE);
                _DEV_MC[HLK].RBJoint_SetMoveJoint(QJ.HLK, ready_ms, MOVE_ABSOLUTE);
                _DEV_MC[FRR].RBJoint_SetMoveJoint(QJ.FRR, ready_ms, MOVE_ABSOLUTE);
                _DEV_MC[FRP].RBJoint_SetMoveJoint(QJ.FRP, ready_ms, MOVE_ABSOLUTE);
                _DEV_MC[FRK].RBJoint_SetMoveJoint(QJ.FRK, ready_ms, MOVE_ABSOLUTE);
                _DEV_MC[FLR].RBJoint_SetMoveJoint(QJ.FLR, ready_ms, MOVE_ABSOLUTE);
                _DEV_MC[FLP].RBJoint_SetMoveJoint(QJ.FLP, ready_ms, MOVE_ABSOLUTE);
                _DEV_MC[FLK].RBJoint_SetMoveJoint(QJ.FLK, ready_ms, MOVE_ABSOLUTE);

                usleep(ready_ms * 1e3);
                FILE_LOG(logSUCCESS) << "MOUNTPOS Done.";

                break;
            }
            case GROUNDPOS:
            {
                sharedData->COMMAND.USER_COMMAND = NO_ACT;
                FILE_LOG(logSUCCESS) << "HUBODOG5_GROUNDPOS";
                for(int i=0;i<NO_OF_JOINTS;i++)
                {
                    MY_CONTROL_FB_GAIN[i] = 10;
                    MY_CONTROL_FF_GAIN[i] = 0;
                }

                QuadJoint QJ;
                QJ.HRR = angle_ground2[0];
                QJ.HRP = angle_ground2[1];
                QJ.HRK = angle_ground2[2];
                QJ.HLR = angle_ground2[0];
                QJ.HLP = angle_ground2[1];
                QJ.HLK = angle_ground2[2];
                QJ.FRR = angle_ground2[0];
                QJ.FRP = angle_ground2[1];
                QJ.FRK = angle_ground2[2];
                QJ.FLR = angle_ground2[0];
                QJ.FLP = angle_ground2[1];
                QJ.FLK = angle_ground2[2];

                _DEV_MC[HRR].RBJoint_SetMoveJoint(QJ.HRR, ready_ms, MOVE_ABSOLUTE);
                _DEV_MC[HRP].RBJoint_SetMoveJoint(QJ.HRP, ready_ms, MOVE_ABSOLUTE);
                _DEV_MC[HRK].RBJoint_SetMoveJoint(QJ.HRK, ready_ms, MOVE_ABSOLUTE);
                _DEV_MC[HLR].RBJoint_SetMoveJoint(QJ.HLR, ready_ms, MOVE_ABSOLUTE);
                _DEV_MC[HLP].RBJoint_SetMoveJoint(QJ.HLP, ready_ms, MOVE_ABSOLUTE);
                _DEV_MC[HLK].RBJoint_SetMoveJoint(QJ.HLK, ready_ms, MOVE_ABSOLUTE);
                _DEV_MC[FRR].RBJoint_SetMoveJoint(QJ.FRR, ready_ms, MOVE_ABSOLUTE);
                _DEV_MC[FRP].RBJoint_SetMoveJoint(QJ.FRP, ready_ms, MOVE_ABSOLUTE);
                _DEV_MC[FRK].RBJoint_SetMoveJoint(QJ.FRK, ready_ms, MOVE_ABSOLUTE);
                _DEV_MC[FLR].RBJoint_SetMoveJoint(QJ.FLR, ready_ms, MOVE_ABSOLUTE);
                _DEV_MC[FLP].RBJoint_SetMoveJoint(QJ.FLP, ready_ms, MOVE_ABSOLUTE);
                _DEV_MC[FLK].RBJoint_SetMoveJoint(QJ.FLK, ready_ms, MOVE_ABSOLUTE);

                usleep(ready_ms * 1e3);

                usleep(100 * 1e3);

                QJ.HRR = angle_ground[0];
                QJ.HRP = angle_ground[1];
                QJ.HRK = angle_ground[2];
                QJ.HLR = angle_ground[0];
                QJ.HLP = angle_ground[1];
                QJ.HLK = angle_ground[2];
                QJ.FRR = angle_ground[0];
                QJ.FRP = angle_ground[1];
                QJ.FRK = angle_ground[2];
                QJ.FLR = angle_ground[0];
                QJ.FLP = angle_ground[1];
                QJ.FLK = angle_ground[2];

                _DEV_MC[HRR].RBJoint_SetMoveJoint(QJ.HRR, ready_ms*2, MOVE_ABSOLUTE);
                _DEV_MC[HRP].RBJoint_SetMoveJoint(QJ.HRP, ready_ms*2, MOVE_ABSOLUTE);
                _DEV_MC[HRK].RBJoint_SetMoveJoint(QJ.HRK, ready_ms*2, MOVE_ABSOLUTE);
                _DEV_MC[HLR].RBJoint_SetMoveJoint(QJ.HLR, ready_ms*2, MOVE_ABSOLUTE);
                _DEV_MC[HLP].RBJoint_SetMoveJoint(QJ.HLP, ready_ms*2, MOVE_ABSOLUTE);
                _DEV_MC[HLK].RBJoint_SetMoveJoint(QJ.HLK, ready_ms*2, MOVE_ABSOLUTE);
                _DEV_MC[FRR].RBJoint_SetMoveJoint(QJ.FRR, ready_ms*2, MOVE_ABSOLUTE);
                _DEV_MC[FRP].RBJoint_SetMoveJoint(QJ.FRP, ready_ms*2, MOVE_ABSOLUTE);
                _DEV_MC[FRK].RBJoint_SetMoveJoint(QJ.FRK, ready_ms*2, MOVE_ABSOLUTE);
                _DEV_MC[FLR].RBJoint_SetMoveJoint(QJ.FLR, ready_ms*2, MOVE_ABSOLUTE);
                _DEV_MC[FLP].RBJoint_SetMoveJoint(QJ.FLP, ready_ms*2, MOVE_ABSOLUTE);
                _DEV_MC[FLK].RBJoint_SetMoveJoint(QJ.FLK, ready_ms*2, MOVE_ABSOLUTE);

                usleep(ready_ms * 1e3);
                FILE_LOG(logSUCCESS) << "GROUNDPOS Done.";

                break;
            }

            case GREADYPOS:
            {
                sharedData->COMMAND.USER_COMMAND = NO_ACT;
                FILE_LOG(logSUCCESS) << "HUBODOG5_GREADYPOS";
                for(int i=0;i<NO_OF_JOINTS;i++)
                {
                    MY_CONTROL_FB_GAIN[i] = 10;
                    MY_CONTROL_FF_GAIN[i] = 0;
                }

                QuadJoint QJ;
                QJ.HRR = angle_ground2[0];
                QJ.HRP = angle_ground2[1];
                QJ.HRK = angle_ground2[2];
                QJ.HLR = angle_ground2[0];
                QJ.HLP = angle_ground2[1];
                QJ.HLK = angle_ground2[2];
                QJ.FRR = angle_ground2[0];
                QJ.FRP = angle_ground2[1];
                QJ.FRK = angle_ground2[2];
                QJ.FLR = angle_ground2[0];
                QJ.FLP = angle_ground2[1];
                QJ.FLK = angle_ground2[2];

                _DEV_MC[HRR].RBJoint_SetMoveJoint(QJ.HRR, ready_ms, MOVE_ABSOLUTE);
                _DEV_MC[HRP].RBJoint_SetMoveJoint(QJ.HRP, ready_ms, MOVE_ABSOLUTE);
                _DEV_MC[HRK].RBJoint_SetMoveJoint(QJ.HRK, ready_ms, MOVE_ABSOLUTE);
                _DEV_MC[HLR].RBJoint_SetMoveJoint(QJ.HLR, ready_ms, MOVE_ABSOLUTE);
                _DEV_MC[HLP].RBJoint_SetMoveJoint(QJ.HLP, ready_ms, MOVE_ABSOLUTE);
                _DEV_MC[HLK].RBJoint_SetMoveJoint(QJ.HLK, ready_ms, MOVE_ABSOLUTE);
                _DEV_MC[FRR].RBJoint_SetMoveJoint(QJ.FRR, ready_ms, MOVE_ABSOLUTE);
                _DEV_MC[FRP].RBJoint_SetMoveJoint(QJ.FRP, ready_ms, MOVE_ABSOLUTE);
                _DEV_MC[FRK].RBJoint_SetMoveJoint(QJ.FRK, ready_ms, MOVE_ABSOLUTE);
                _DEV_MC[FLR].RBJoint_SetMoveJoint(QJ.FLR, ready_ms, MOVE_ABSOLUTE);
                _DEV_MC[FLP].RBJoint_SetMoveJoint(QJ.FLP, ready_ms, MOVE_ABSOLUTE);
                _DEV_MC[FLK].RBJoint_SetMoveJoint(QJ.FLK, ready_ms, MOVE_ABSOLUTE);

                usleep(ready_ms * 1e3);

                usleep(100 * 1e3);

                QJ.HRR = _DEV_MC[HRR].homeJointReady;
                QJ.HRP = _DEV_MC[HRP].homeJointReady;
                QJ.HRK = _DEV_MC[HRK].homeJointReady;
                QJ.HLR = _DEV_MC[HLR].homeJointReady;
                QJ.HLP = _DEV_MC[HLP].homeJointReady;
                QJ.HLK = _DEV_MC[HLK].homeJointReady;
                QJ.FRR = _DEV_MC[FRR].homeJointReady;
                QJ.FRP = _DEV_MC[FRP].homeJointReady;
                QJ.FRK = _DEV_MC[FRK].homeJointReady;
                QJ.FLR = _DEV_MC[FLR].homeJointReady;
                QJ.FLP = _DEV_MC[FLP].homeJointReady;
                QJ.FLK = _DEV_MC[FLK].homeJointReady;

                _DEV_MC[HRR].RBJoint_SetMoveJoint(QJ.HRR, ready_ms, MOVE_ABSOLUTE);
                _DEV_MC[HRP].RBJoint_SetMoveJoint(QJ.HRP, ready_ms, MOVE_ABSOLUTE);
                _DEV_MC[HRK].RBJoint_SetMoveJoint(QJ.HRK, ready_ms, MOVE_ABSOLUTE);
                _DEV_MC[HLR].RBJoint_SetMoveJoint(QJ.HLR, ready_ms, MOVE_ABSOLUTE);
                _DEV_MC[HLP].RBJoint_SetMoveJoint(QJ.HLP, ready_ms, MOVE_ABSOLUTE);
                _DEV_MC[HLK].RBJoint_SetMoveJoint(QJ.HLK, ready_ms, MOVE_ABSOLUTE);
                _DEV_MC[FRR].RBJoint_SetMoveJoint(QJ.FRR, ready_ms, MOVE_ABSOLUTE);
                _DEV_MC[FRP].RBJoint_SetMoveJoint(QJ.FRP, ready_ms, MOVE_ABSOLUTE);
                _DEV_MC[FRK].RBJoint_SetMoveJoint(QJ.FRK, ready_ms, MOVE_ABSOLUTE);
                _DEV_MC[FLR].RBJoint_SetMoveJoint(QJ.FLR, ready_ms, MOVE_ABSOLUTE);
                _DEV_MC[FLP].RBJoint_SetMoveJoint(QJ.FLP, ready_ms, MOVE_ABSOLUTE);
                _DEV_MC[FLK].RBJoint_SetMoveJoint(QJ.FLK, ready_ms, MOVE_ABSOLUTE);

                usleep(ready_ms * 1e3);
                FILE_LOG(logSUCCESS) << "GREADYPOS Done.";

                Quad.isReadyDone = true;

                break;
            }
            case HOMMINGPOS:
            {
                sharedData->COMMAND.USER_COMMAND = NO_ACT;
                FILE_LOG(logSUCCESS) << "HUBODOG5_HOMMINGPOS";
                for(int i=0;i<NO_OF_JOINTS;i++)
                {
                    MY_CONTROL_FB_GAIN[i] = 10;
                    MY_CONTROL_FF_GAIN[i] = 0;
                }

                QuadJoint QJ;
                double kn_offset = 9.83;

                QJ.HRR = 0;
                QJ.HRP = 180.0;
                QJ.HRK = (-90.0+kn_offset) - 180/11.0;
                QJ.HLR = 0;
                QJ.HLP = 180.0;
                QJ.HLK = (-90.0+kn_offset) - 180/11.0;
                QJ.FRR = 0;
                QJ.FRP = 180.0;
                QJ.FRK = (-90.0+kn_offset) - 180/11.0;
                QJ.FLR = 0;
                QJ.FLP = 180.0;
                QJ.FLK = (-90.0+kn_offset) - 180/11.0;

                _DEV_MC[HRR].RBJoint_SetMoveJoint(QJ.HRR, ready_ms, MOVE_ABSOLUTE);
                _DEV_MC[HRP].RBJoint_SetMoveJoint(QJ.HRP, ready_ms, MOVE_ABSOLUTE);
                _DEV_MC[HRK].RBJoint_SetMoveJoint(QJ.HRK, ready_ms, MOVE_ABSOLUTE);
                _DEV_MC[HLR].RBJoint_SetMoveJoint(QJ.HLR, ready_ms, MOVE_ABSOLUTE);
                _DEV_MC[HLP].RBJoint_SetMoveJoint(QJ.HLP, ready_ms, MOVE_ABSOLUTE);
                _DEV_MC[HLK].RBJoint_SetMoveJoint(QJ.HLK, ready_ms, MOVE_ABSOLUTE);
                _DEV_MC[FRR].RBJoint_SetMoveJoint(QJ.FRR, ready_ms, MOVE_ABSOLUTE);
                _DEV_MC[FRP].RBJoint_SetMoveJoint(QJ.FRP, ready_ms, MOVE_ABSOLUTE);
                _DEV_MC[FRK].RBJoint_SetMoveJoint(QJ.FRK, ready_ms, MOVE_ABSOLUTE);
                _DEV_MC[FLR].RBJoint_SetMoveJoint(QJ.FLR, ready_ms, MOVE_ABSOLUTE);
                _DEV_MC[FLP].RBJoint_SetMoveJoint(QJ.FLP, ready_ms, MOVE_ABSOLUTE);
                _DEV_MC[FLK].RBJoint_SetMoveJoint(QJ.FLK, ready_ms, MOVE_ABSOLUTE);

                usleep(ready_ms * 1e3);
                FILE_LOG(logSUCCESS) << "HOMMINGPOS Done.";

                Quad.isReadyDone = true;

                break;
            }
            case JOGMODE:
            {
                sharedData->COMMAND.USER_COMMAND = NO_ACT;
                double joint_vel = sharedData->COMMAND.USER_PARA_DOUBLE[0]*10;

        //            Jog_ref_flag = 0;
        //        }
                for(int i=0; i<12; i++) {
                    if(sharedData->jog_joint[i] == true) angle_rel[i] = joint_vel;
                    else angle_rel[i] = 0;
                }
                for(int i=0;i<NO_OF_JOINTS;i++)
                {
                    MY_CONTROL_FB_GAIN[i] = 10;
                    MY_CONTROL_FF_GAIN[i] = 0;
                }

                QuadJoint QJ;
                QJ.HRR = GetOneJointPosRef(HRR) + angle_rel[HRR];
                QJ.HRP = GetOneJointPosRef(HRP) + angle_rel[HRP];
                QJ.HRK = GetOneJointPosRef(HRK) + angle_rel[HRK];
                QJ.HLR = GetOneJointPosRef(HLR) + angle_rel[HLR];
                QJ.HLP = GetOneJointPosRef(HLP) + angle_rel[HLP];
                QJ.HLK = GetOneJointPosRef(HLK) + angle_rel[HLK];
                QJ.FRR = GetOneJointPosRef(FRR) + angle_rel[FRR];
                QJ.FRP = GetOneJointPosRef(FRP) + angle_rel[FRP];
                QJ.FRK = GetOneJointPosRef(FRK) + angle_rel[FRK];
                QJ.FLR = GetOneJointPosRef(FLR) + angle_rel[FLR];
                QJ.FLP = GetOneJointPosRef(FLP) + angle_rel[FLP];
                QJ.FLK = GetOneJointPosRef(FLK) + angle_rel[FLK];

                _DEV_MC[HRR].RBJoint_SetMoveJoint(QJ.HRR*R2Df, 10, MOVE_ABSOLUTE);
                _DEV_MC[HRP].RBJoint_SetMoveJoint(QJ.HRP*R2Df, 10, MOVE_ABSOLUTE);
                _DEV_MC[HRK].RBJoint_SetMoveJoint(QJ.HRK*R2Df, 10, MOVE_ABSOLUTE);
                _DEV_MC[HLR].RBJoint_SetMoveJoint(QJ.HLR*R2Df, 10, MOVE_ABSOLUTE);
                _DEV_MC[HLP].RBJoint_SetMoveJoint(QJ.HLP*R2Df, 10, MOVE_ABSOLUTE);
                _DEV_MC[HLK].RBJoint_SetMoveJoint(QJ.HLK*R2Df, 10, MOVE_ABSOLUTE);
                _DEV_MC[FRR].RBJoint_SetMoveJoint(QJ.FRR*R2Df, 10, MOVE_ABSOLUTE);
                _DEV_MC[FRP].RBJoint_SetMoveJoint(QJ.FRP*R2Df, 10, MOVE_ABSOLUTE);
                _DEV_MC[FRK].RBJoint_SetMoveJoint(QJ.FRK*R2Df, 10, MOVE_ABSOLUTE);
                _DEV_MC[FLR].RBJoint_SetMoveJoint(QJ.FLR*R2Df, 10, MOVE_ABSOLUTE);
                _DEV_MC[FLP].RBJoint_SetMoveJoint(QJ.FLP*R2Df, 10, MOVE_ABSOLUTE);
                _DEV_MC[FLK].RBJoint_SetMoveJoint(QJ.FLK*R2Df, 10, MOVE_ABSOLUTE);


//                if(jog_mode == 0) idx = NO_ACT;
//                else {
////                    if(_DEV_MC[MAX_MC-1].MoveJoints.MoveFlag == 0) {
////                        idx = NO_ACT;
////                    }
//                }
                break;

            }
            case MOTION_BACKFLIP:
            {

                sharedData->COMMAND.USER_COMMAND = NO_ACT;
                if(Quad.isReadyDone)
                {
                    Quad.isReadyDone = false;
                    FILE_LOG(logINFO) << "HUBODOG5_BACKFLIP";


                    QuadJoint QJ;
                    double t_target;

                    for(int i=0;i<NO_OF_JOINTS;i++)
                    {
                        MY_CONTROL_FB_GAIN[i] = 10;
                        MY_CONTROL_FF_GAIN[i] = 0;
                    }

                    if(sharedData->REF_Enabled == true) {
                        for(int i=0; i<100; i++) {
                            for(int j=0; j<12; j++) {_DEV_MC[j].RBFOC_SetGain_POS(i+200,(int)(i/5),600, 1);}

                            if(i < 50) usleep((70-i) * 1e3);
                            else if(i < 100) usleep(2 * 1e3);
                            else usleep(5 * 1e3);

                        }
                        for(int j=0; j<12; j++) {_DEV_MC[j].RBFOC_SetGain_POS(300,20,600, 0);}
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

                        t_target = (60-5+5)*t_temp; // loger time more rotation

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
                        if(sharedData->REF_Enabled == true) {
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
                        }


                        //double roll_end = 7.5;
                        double roll_end = 7.5;
                        QJ.HRR = -roll_end;
                        QJ.HLR = roll_end;
                        QJ.FRR = -roll_end;
                        QJ.FLR = roll_end;

                        QJ.HRP = 60;//-10;
                        QJ.HLP = 60;//-10;
                        QJ.HRK = -110 - (QJ.HRP)/11;//+50;
                        QJ.HLK = -110 - (QJ.HLP)/11;//+50;

                        // front
                        QJ.FRP = 60;//-10;
                        QJ.FLP = 60;//-10;
                        QJ.FRK = -110 - (QJ.FRP)/11;//+50;
                        QJ.FLK = -110 - (QJ.FLP)/11;//+50;

                        t_target = (300-100-60)*t_temp;

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

                        usleep(2000 * 1e3);


                        QJ.HRR = 0;
                        QJ.HLR = 0;
                        QJ.FRR = 0;
                        QJ.FLR = 0;

                        t_target = 500;

                        _DEV_MC[HRR].RBJoint_SetMoveJoint(QJ.HRR, t_target, MOVE_ABSOLUTE);
                        _DEV_MC[HLR].RBJoint_SetMoveJoint(QJ.HLR, t_target, MOVE_ABSOLUTE);
                        _DEV_MC[FRR].RBJoint_SetMoveJoint(QJ.FRR, t_target, MOVE_ABSOLUTE);
                        _DEV_MC[FLR].RBJoint_SetMoveJoint(QJ.FLR, t_target, MOVE_ABSOLUTE);
                        usleep(t_target * 1e3);

//                        if(sharedData->REF_Enabled == true) {
//                            for(int i=0; i<150; i++) {
//                                for(int j=0; j<12; j++) {_DEV_MC[j].RBFOC_SetGain_POS(50+i,0,600, 1);}

//                                if(i < 50) usleep(40 * 1e3);
//                                else if(i < 100) usleep(20 * 1e3);
//                                else usleep(5 * 1e3);

//                            }
//                            for(int j=0; j<12; j++) {_DEV_MC[j].RBFOC_SetGain_POS(200,0,600, 0);}
//                        }

                        if(sharedData->REF_Enabled == true) {
                            for(int i=0; i<100; i++) {
                                for(int j=0; j<12; j++) {_DEV_MC[j].RBFOC_SetGain_POS(300-i,0,600, 1);}

                                if(i < 50) usleep(40 * 1e3);
                                else if(i < 100) usleep(20 * 1e3);
                                else usleep(5 * 1e3);

                            }
                            for(int j=0; j<12; j++) {_DEV_MC[j].RBFOC_SetGain_POS(200,0,600, 0);}
                        }

                        QJ.HRR = _DEV_MC[HRR].homeJointReady;
                        QJ.HRP = _DEV_MC[HRP].homeJointReady;
                        QJ.HRK = _DEV_MC[HRK].homeJointReady;
                        QJ.HLR = _DEV_MC[HLR].homeJointReady;
                        QJ.HLP = _DEV_MC[HLP].homeJointReady;
                        QJ.HLK = _DEV_MC[HLK].homeJointReady;
                        QJ.FRR = _DEV_MC[FRR].homeJointReady;
                        QJ.FRP = _DEV_MC[FRP].homeJointReady;
                        QJ.FRK = _DEV_MC[FRK].homeJointReady;
                        QJ.FLR = _DEV_MC[FLR].homeJointReady;
                        QJ.FLP = _DEV_MC[FLP].homeJointReady;
                        QJ.FLK = _DEV_MC[FLK].homeJointReady;

                        _DEV_MC[HRR].RBJoint_SetMoveJoint(QJ.HRR, ready_ms, MOVE_ABSOLUTE);
                        _DEV_MC[HRP].RBJoint_SetMoveJoint(QJ.HRP, ready_ms, MOVE_ABSOLUTE);
                        _DEV_MC[HRK].RBJoint_SetMoveJoint(QJ.HRK, ready_ms, MOVE_ABSOLUTE);
                        _DEV_MC[HLR].RBJoint_SetMoveJoint(QJ.HLR, ready_ms, MOVE_ABSOLUTE);
                        _DEV_MC[HLP].RBJoint_SetMoveJoint(QJ.HLP, ready_ms, MOVE_ABSOLUTE);
                        _DEV_MC[HLK].RBJoint_SetMoveJoint(QJ.HLK, ready_ms, MOVE_ABSOLUTE);
                        _DEV_MC[FRR].RBJoint_SetMoveJoint(QJ.FRR, ready_ms, MOVE_ABSOLUTE);
                        _DEV_MC[FRP].RBJoint_SetMoveJoint(QJ.FRP, ready_ms, MOVE_ABSOLUTE);
                        _DEV_MC[FRK].RBJoint_SetMoveJoint(QJ.FRK, ready_ms, MOVE_ABSOLUTE);
                        _DEV_MC[FLR].RBJoint_SetMoveJoint(QJ.FLR, ready_ms, MOVE_ABSOLUTE);
                        _DEV_MC[FLP].RBJoint_SetMoveJoint(QJ.FLP, ready_ms, MOVE_ABSOLUTE);
                        _DEV_MC[FLK].RBJoint_SetMoveJoint(QJ.FLK, ready_ms, MOVE_ABSOLUTE);


                        usleep(ready_ms * 1e3);


                        Quad.isReadyDone = true;
                        sharedData->isFalled = false;


                        FILE_LOG(logSUCCESS) << "BACK FLIP REAL Done.";
                        FILE_LOG(logSUCCESS) << "READYPOS Done.";
                    }

                }
                else{
                    FILE_LOG(logWARNING) << "Please READYPOS First.";
                }

                break;
            }

            case MOTION_BACKFLIP_H:
            {

                sharedData->COMMAND.USER_COMMAND = NO_ACT;
                if(Quad.isReadyDone)
                {
                    Quad.isReadyDone = false;
                    FILE_LOG(logINFO) << "HUBODOG5_BACKFLIP";


                    QuadJoint QJ;
                    double t_target, t_target_hind;

                    for(int i=0;i<NO_OF_JOINTS;i++)
                    {
                        MY_CONTROL_FB_GAIN[i] = 10;
                        MY_CONTROL_FF_GAIN[i] = 0;
                    }

                    if(sharedData->REF_Enabled == true) {
                        for(int i=0; i<100; i++) {
                            for(int j=0; j<12; j++) {_DEV_MC[j].RBFOC_SetGain_POS(i+200,(int)(i/5),600, 1);}

                            if(i < 50) usleep((70-i) * 1e3);
                            else if(i < 100) usleep(2 * 1e3);
                            else usleep(5 * 1e3);

                        }
                        for(int j=0; j<12; j++) {_DEV_MC[j].RBFOC_SetGain_POS(300,20,600, 0);}
                    }


                    if(sharedData->RaisimSimulation == true) {
                        QJ.HRR = 0;
                        QJ.HLR = 0;
                        QJ.FRR = 0;
                        QJ.FLR = 0;

                        // 1. down

                        int no_of_turn = _DEV_MC[HRP].MoveJoints.RefAngleCurrent/180.0;

                        cout<<"HRP: "<<_DEV_MC[HRP].MoveJoints.RefAngleCurrent<<"  currunt turn: "<<no_of_turn<<endl;

                        if(no_of_turn%2 == 0){
                            // hind
                            QJ.HRP = QJ.HLP = 35 + 180.0*no_of_turn;
                            QJ.HRK = QJ.HLK = -100 - (QJ.HRP/11.0);

                            //front
                            QJ.FRP = QJ.FLP = 35 +180.0*no_of_turn;
                            QJ.FRK = QJ.FLK = -100 - (QJ.FRP/11.0);
                        }
                        else{
                            // hind
                            QJ.HRP = QJ.HLP = 35 +180.0*no_of_turn;
                            QJ.HRK = QJ.HLK = -100 - (QJ.HRP/11.0);

                            //front
                            QJ.FRP = QJ.FLP = 35 + 180.0*no_of_turn;
                            QJ.FRK = QJ.FLK = -100 - (QJ.FRP/11.0);
                        }


                        t_target = 600;

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
                        if(no_of_turn%2 == 0){
                            // hind
                            QJ.HRP = QJ.HLP = 50 +180.0*no_of_turn;
                            QJ.HRK = QJ.HLK = -100 - (QJ.HRP/11.0);

                            // front
                            QJ.FRP = QJ.FLP = 25 + 180.0*no_of_turn;
                            QJ.FRK = QJ.FLK = -50 - (QJ.FRP/11.0);
                        }
                        else{
                            // hind
                            QJ.HRP = QJ.HLP = 25 + 180.0*no_of_turn;
                            QJ.HRK = QJ.HLK = -50 - (QJ.HRP/11.0);

                            // front
                            QJ.FRP = QJ.FLP = 50 +180.0*no_of_turn;
                            QJ.FRK = QJ.FLK = -100 - (QJ.FRP/11.0);
                        }

                        t_target = 90;

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

                        // 3. front and hind rotation
                        if(no_of_turn%2 == 0){
                            // hind walkready
                            QJ.HRP = QJ.HLP = 55+180 + 180.0*no_of_turn;
                            QJ.HRK = QJ.HLK = -100 - (QJ.HRP/11.0);
                            t_target_hind = 700;

                            // front
                            QJ.FRP = QJ.FLP = 180 +180.0*no_of_turn;
                            QJ.FRK = QJ.FLK = -140 - (QJ.FRP/11.0);
                            t_target = 350;
                            _DEV_MC[HRR].RBJoint_SetMoveJoint(QJ.HRR, t_target_hind, MOVE_ABSOLUTE);
                            _DEV_MC[HRP].RBJoint_SetMoveJoint(QJ.HRP, t_target_hind, MOVE_ABSOLUTE);
                            _DEV_MC[HRK].RBJoint_SetMoveJoint(QJ.HRK, t_target_hind, MOVE_ABSOLUTE);
                            _DEV_MC[HLR].RBJoint_SetMoveJoint(QJ.HLR, t_target_hind, MOVE_ABSOLUTE);
                            _DEV_MC[HLP].RBJoint_SetMoveJoint(QJ.HLP, t_target_hind, MOVE_ABSOLUTE);
                            _DEV_MC[HLK].RBJoint_SetMoveJoint(QJ.HLK, t_target_hind, MOVE_ABSOLUTE);
                            _DEV_MC[FRR].RBJoint_SetMoveJoint(QJ.FRR, t_target, MOVE_ABSOLUTE);
                            _DEV_MC[FRP].RBJoint_SetMoveJoint(QJ.FRP, t_target, MOVE_ABSOLUTE);
                            _DEV_MC[FRK].RBJoint_SetMoveJoint(QJ.FRK, t_target, MOVE_ABSOLUTE);
                            _DEV_MC[FLR].RBJoint_SetMoveJoint(QJ.FLR, t_target, MOVE_ABSOLUTE);
                            _DEV_MC[FLP].RBJoint_SetMoveJoint(QJ.FLP, t_target, MOVE_ABSOLUTE);
                            _DEV_MC[FLK].RBJoint_SetMoveJoint(QJ.FLK, t_target, MOVE_ABSOLUTE);
                            usleep(t_target * 1e3);

                        }
                        else{
                            // hind walkready
                            QJ.HRP = QJ.HLP = 180 +180.0*no_of_turn;
                            QJ.HRK = QJ.HLK = -140 - (QJ.HRP/11.0);
                            t_target_hind = 350;

                            // front
                            QJ.FRP = QJ.FLP = 55+180 + 180.0*no_of_turn;
                            QJ.FRK = QJ.FLK = -100 - (QJ.FRP/11.0);
                            t_target = 700;
                            _DEV_MC[HRR].RBJoint_SetMoveJoint(QJ.HRR, t_target_hind, MOVE_ABSOLUTE);
                            _DEV_MC[HRP].RBJoint_SetMoveJoint(QJ.HRP, t_target_hind, MOVE_ABSOLUTE);
                            _DEV_MC[HRK].RBJoint_SetMoveJoint(QJ.HRK, t_target_hind, MOVE_ABSOLUTE);
                            _DEV_MC[HLR].RBJoint_SetMoveJoint(QJ.HLR, t_target_hind, MOVE_ABSOLUTE);
                            _DEV_MC[HLP].RBJoint_SetMoveJoint(QJ.HLP, t_target_hind, MOVE_ABSOLUTE);
                            _DEV_MC[HLK].RBJoint_SetMoveJoint(QJ.HLK, t_target_hind, MOVE_ABSOLUTE);
                            _DEV_MC[FRR].RBJoint_SetMoveJoint(QJ.FRR, t_target, MOVE_ABSOLUTE);
                            _DEV_MC[FRP].RBJoint_SetMoveJoint(QJ.FRP, t_target, MOVE_ABSOLUTE);
                            _DEV_MC[FRK].RBJoint_SetMoveJoint(QJ.FRK, t_target, MOVE_ABSOLUTE);
                            _DEV_MC[FLR].RBJoint_SetMoveJoint(QJ.FLR, t_target, MOVE_ABSOLUTE);
                            _DEV_MC[FLP].RBJoint_SetMoveJoint(QJ.FLP, t_target, MOVE_ABSOLUTE);
                            _DEV_MC[FLK].RBJoint_SetMoveJoint(QJ.FLK, t_target, MOVE_ABSOLUTE);
                            usleep(t_target_hind * 1e3);

                        }







                        // 4. return to walk ready
                        if(no_of_turn%2 == 0){

                            // hind
                            //QJ.HRP = QJ.HLP = 50+180;
                            //QJ.HRK = QJ.HLK = -130;


                            // front
                            QJ.FRP = QJ.FLP = 55+180 +180.0*no_of_turn;
                            QJ.FRK = QJ.FLK = -100 - (QJ.FRP/11.0);


                            t_target = 350;

    //                        _DEV_MC[HRR].RBJoint_SetMoveJoint(QJ.HRR, t_target, MOVE_ABSOLUTE);
    //                        _DEV_MC[HRP].RBJoint_SetMoveJoint(QJ.HRP, t_target, MOVE_ABSOLUTE);
    //                        _DEV_MC[HRK].RBJoint_SetMoveJoint(QJ.HRK, t_target, MOVE_ABSOLUTE);
    //                        _DEV_MC[HLR].RBJoint_SetMoveJoint(QJ.HLR, t_target, MOVE_ABSOLUTE);
    //                        _DEV_MC[HLP].RBJoint_SetMoveJoint(QJ.HLP, t_target, MOVE_ABSOLUTE);
    //                        _DEV_MC[HLK].RBJoint_SetMoveJoint(QJ.HLK, t_target, MOVE_ABSOLUTE);
                            _DEV_MC[FRR].RBJoint_SetMoveJoint(QJ.FRR, t_target, MOVE_ABSOLUTE);
                            _DEV_MC[FRP].RBJoint_SetMoveJoint(QJ.FRP, t_target, MOVE_ABSOLUTE);
                            _DEV_MC[FRK].RBJoint_SetMoveJoint(QJ.FRK, t_target, MOVE_ABSOLUTE);
                            _DEV_MC[FLR].RBJoint_SetMoveJoint(QJ.FLR, t_target, MOVE_ABSOLUTE);
                            _DEV_MC[FLP].RBJoint_SetMoveJoint(QJ.FLP, t_target, MOVE_ABSOLUTE);
                            _DEV_MC[FLK].RBJoint_SetMoveJoint(QJ.FLK, t_target, MOVE_ABSOLUTE);
                        }
                        else{
                            // 4. return to walk ready
                            // hind
                            QJ.HRP = QJ.HLP = 55+180 +180.0*no_of_turn;
                            QJ.HRK = QJ.HLK = -100 - (QJ.HRP/11.0);


                            // front
                            //QJ.FRP = QJ.FLP = 55+180 +180.0*no_of_turn;
                            //QJ.FRK = QJ.FLK = -100 - (QJ.FRP/11.0);


                            t_target_hind = 350;

                            _DEV_MC[HRR].RBJoint_SetMoveJoint(QJ.HRR, t_target_hind, MOVE_ABSOLUTE);
                            _DEV_MC[HRP].RBJoint_SetMoveJoint(QJ.HRP, t_target_hind, MOVE_ABSOLUTE);
                            _DEV_MC[HRK].RBJoint_SetMoveJoint(QJ.HRK, t_target_hind, MOVE_ABSOLUTE);
                            _DEV_MC[HLR].RBJoint_SetMoveJoint(QJ.HLR, t_target_hind, MOVE_ABSOLUTE);
                            _DEV_MC[HLP].RBJoint_SetMoveJoint(QJ.HLP, t_target_hind, MOVE_ABSOLUTE);
                            _DEV_MC[HLK].RBJoint_SetMoveJoint(QJ.HLK, t_target_hind, MOVE_ABSOLUTE);
//                            _DEV_MC[FRR].RBJoint_SetMoveJoint(QJ.FRR, t_targ_hindet, MOVE_ABSOLUTE);
//                            _DEV_MC[FRP].RBJoint_SetMoveJoint(QJ.FRP, t_target, MOVE_ABSOLUTE);
//                            _DEV_MC[FRK].RBJoint_SetMoveJoint(QJ.FRK, t_target, MOVE_ABSOLUTE);
//                            _DEV_MC[FLR].RBJoint_SetMoveJoint(QJ.FLR, t_target, MOVE_ABSOLUTE);
//                            _DEV_MC[FLP].RBJoint_SetMoveJoint(QJ.FLP, t_target, MOVE_ABSOLUTE);
//                            _DEV_MC[FLK].RBJoint_SetMoveJoint(QJ.FLK, t_target, MOVE_ABSOLUTE);
                        }


                        usleep(t_target * 1e3);

                        Quad.isReadyDone = true;

                        FILE_LOG(logSUCCESS) << "BACK FLIP SIMULATION Done.";
                    }
                    else {
                        QJ.HRR = 0;
                        QJ.HLR = 0;
                        QJ.FRR = 0;
                        QJ.FLR = 0;

                        // 1. down

                        int no_of_turn = _DEV_MC[HRP].MoveJoints.RefAngleCurrent/180.0;

                        cout<<"HRP: "<<_DEV_MC[HRP].MoveJoints.RefAngleCurrent<<"  currunt turn: "<<no_of_turn<<endl;

                        if(no_of_turn%2 == 0){
                            // hind
                            QJ.HRP = QJ.HLP = 35 + 180.0*no_of_turn;
                            QJ.HRK = QJ.HLK = -100 - (QJ.HRP/11.0);

                            //front
                            QJ.FRP = QJ.FLP = 35 +180.0*no_of_turn;
                            QJ.FRK = QJ.FLK = -100 - (QJ.FRP/11.0);
                        }
                        else{
                            // hind
                            QJ.HRP = QJ.HLP = 35 +180.0*no_of_turn;
                            QJ.HRK = QJ.HLK = -100 - (QJ.HRP/11.0);

                            //front
                            QJ.FRP = QJ.FLP = 35 + 180.0*no_of_turn;
                            QJ.FRK = QJ.FLK = -100 - (QJ.FRP/11.0);
                        }


                        t_target = 600;

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
                        if(no_of_turn%2 == 0){
                            // hind
                            QJ.HRP = QJ.HLP = 45 +180.0*no_of_turn;
                            QJ.HRK = QJ.HLK = -100 - (QJ.HRP/11.0);

                            // front
                            QJ.FRP = QJ.FLP = 25 + 180.0*no_of_turn;
                            QJ.FRK = QJ.FLK = -50 - (QJ.FRP/11.0);
                        }
                        else{
                            // hind
                            QJ.HRP = QJ.HLP = 25 + 180.0*no_of_turn;
                            QJ.HRK = QJ.HLK = -50 - (QJ.HRP/11.0);

                            // front
                            QJ.FRP = QJ.FLP = 45 +180.0*no_of_turn;
                            QJ.FRK = QJ.FLK = -100 - (QJ.FRP/11.0);
                        }

                        t_target = 90;

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

                        // 3. front and hind rotation
                        if(no_of_turn%2 == 0){
                            // hind walkready
                            QJ.HRP = QJ.HLP = 55+180 + 180.0*no_of_turn;
                            QJ.HRK = QJ.HLK = -100 - (QJ.HRP/11.0);
                            t_target_hind = 700+50;

                            // front
                            QJ.FRP = QJ.FLP = 180 +180.0*no_of_turn;
                            QJ.FRK = QJ.FLK = -140 - (QJ.FRP/11.0);
                            t_target = 350;
                            _DEV_MC[HRR].RBJoint_SetMoveJoint(QJ.HRR, t_target_hind, MOVE_ABSOLUTE);
                            _DEV_MC[HRP].RBJoint_SetMoveJoint(QJ.HRP, t_target_hind, MOVE_ABSOLUTE);
                            _DEV_MC[HRK].RBJoint_SetMoveJoint(QJ.HRK, t_target_hind, MOVE_ABSOLUTE);
                            _DEV_MC[HLR].RBJoint_SetMoveJoint(QJ.HLR, t_target_hind, MOVE_ABSOLUTE);
                            _DEV_MC[HLP].RBJoint_SetMoveJoint(QJ.HLP, t_target_hind, MOVE_ABSOLUTE);
                            _DEV_MC[HLK].RBJoint_SetMoveJoint(QJ.HLK, t_target_hind, MOVE_ABSOLUTE);
                            _DEV_MC[FRR].RBJoint_SetMoveJoint(QJ.FRR, t_target, MOVE_ABSOLUTE);
                            _DEV_MC[FRP].RBJoint_SetMoveJoint(QJ.FRP, t_target, MOVE_ABSOLUTE);
                            _DEV_MC[FRK].RBJoint_SetMoveJoint(QJ.FRK, t_target, MOVE_ABSOLUTE);
                            _DEV_MC[FLR].RBJoint_SetMoveJoint(QJ.FLR, t_target, MOVE_ABSOLUTE);
                            _DEV_MC[FLP].RBJoint_SetMoveJoint(QJ.FLP, t_target, MOVE_ABSOLUTE);
                            _DEV_MC[FLK].RBJoint_SetMoveJoint(QJ.FLK, t_target, MOVE_ABSOLUTE);
                            usleep(t_target * 1e3);

                        }
                        else{
                            // hind walkready
                            QJ.HRP = QJ.HLP = 180 +180.0*no_of_turn;
                            QJ.HRK = QJ.HLK = -140 - (QJ.HRP/11.0);
                            t_target_hind = 350;

                            // front
                            QJ.FRP = QJ.FLP = 55+180 + 180.0*no_of_turn;
                            QJ.FRK = QJ.FLK = -100 - (QJ.FRP/11.0);
                            t_target = 700+50;
                            _DEV_MC[HRR].RBJoint_SetMoveJoint(QJ.HRR, t_target_hind, MOVE_ABSOLUTE);
                            _DEV_MC[HRP].RBJoint_SetMoveJoint(QJ.HRP, t_target_hind, MOVE_ABSOLUTE);
                            _DEV_MC[HRK].RBJoint_SetMoveJoint(QJ.HRK, t_target_hind, MOVE_ABSOLUTE);
                            _DEV_MC[HLR].RBJoint_SetMoveJoint(QJ.HLR, t_target_hind, MOVE_ABSOLUTE);
                            _DEV_MC[HLP].RBJoint_SetMoveJoint(QJ.HLP, t_target_hind, MOVE_ABSOLUTE);
                            _DEV_MC[HLK].RBJoint_SetMoveJoint(QJ.HLK, t_target_hind, MOVE_ABSOLUTE);
                            _DEV_MC[FRR].RBJoint_SetMoveJoint(QJ.FRR, t_target, MOVE_ABSOLUTE);
                            _DEV_MC[FRP].RBJoint_SetMoveJoint(QJ.FRP, t_target, MOVE_ABSOLUTE);
                            _DEV_MC[FRK].RBJoint_SetMoveJoint(QJ.FRK, t_target, MOVE_ABSOLUTE);
                            _DEV_MC[FLR].RBJoint_SetMoveJoint(QJ.FLR, t_target, MOVE_ABSOLUTE);
                            _DEV_MC[FLP].RBJoint_SetMoveJoint(QJ.FLP, t_target, MOVE_ABSOLUTE);
                            _DEV_MC[FLK].RBJoint_SetMoveJoint(QJ.FLK, t_target, MOVE_ABSOLUTE);
                            usleep(t_target_hind * 1e3);

                        }

                        if(sharedData->REF_Enabled == true) {
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
                        }







                        // 4. return to walk ready
                        if(no_of_turn%2 == 0){

                            // hind
                            //QJ.HRP = QJ.HLP = 50+180;
                            //QJ.HRK = QJ.HLK = -130;


                            // front
                            QJ.FRP = QJ.FLP = 55+180 +180.0*no_of_turn;
                            QJ.FRK = QJ.FLK = -100 - (QJ.FRP/11.0);


                            t_target = 100;

    //                        _DEV_MC[HRR].RBJoint_SetMoveJoint(QJ.HRR, t_target, MOVE_ABSOLUTE);
    //                        _DEV_MC[HRP].RBJoint_SetMoveJoint(QJ.HRP, t_target, MOVE_ABSOLUTE);
    //                        _DEV_MC[HRK].RBJoint_SetMoveJoint(QJ.HRK, t_target, MOVE_ABSOLUTE);
    //                        _DEV_MC[HLR].RBJoint_SetMoveJoint(QJ.HLR, t_target, MOVE_ABSOLUTE);
    //                        _DEV_MC[HLP].RBJoint_SetMoveJoint(QJ.HLP, t_target, MOVE_ABSOLUTE);
    //                        _DEV_MC[HLK].RBJoint_SetMoveJoint(QJ.HLK, t_target, MOVE_ABSOLUTE);
                            _DEV_MC[FRR].RBJoint_SetMoveJoint(QJ.FRR, t_target, MOVE_ABSOLUTE);
                            _DEV_MC[FRP].RBJoint_SetMoveJoint(QJ.FRP, t_target, MOVE_ABSOLUTE);
                            _DEV_MC[FRK].RBJoint_SetMoveJoint(QJ.FRK, t_target, MOVE_ABSOLUTE);
                            _DEV_MC[FLR].RBJoint_SetMoveJoint(QJ.FLR, t_target, MOVE_ABSOLUTE);
                            _DEV_MC[FLP].RBJoint_SetMoveJoint(QJ.FLP, t_target, MOVE_ABSOLUTE);
                            _DEV_MC[FLK].RBJoint_SetMoveJoint(QJ.FLK, t_target, MOVE_ABSOLUTE);

                            usleep(t_target * 1e3);
                        }
                        else{
                            // 4. return to walk ready
                            // hind
                            QJ.HRP = QJ.HLP = 55+180 +180.0*no_of_turn;
                            QJ.HRK = QJ.HLK = -100 - (QJ.HRP/11.0);


                            // front
                            //QJ.FRP = QJ.FLP = 55+180 +180.0*no_of_turn;
                            //QJ.FRK = QJ.FLK = -100 - (QJ.FRP/11.0);


                            t_target_hind = 100;

                            _DEV_MC[HRR].RBJoint_SetMoveJoint(QJ.HRR, t_target_hind, MOVE_ABSOLUTE);
                            _DEV_MC[HRP].RBJoint_SetMoveJoint(QJ.HRP, t_target_hind, MOVE_ABSOLUTE);
                            _DEV_MC[HRK].RBJoint_SetMoveJoint(QJ.HRK, t_target_hind, MOVE_ABSOLUTE);
                            _DEV_MC[HLR].RBJoint_SetMoveJoint(QJ.HLR, t_target_hind, MOVE_ABSOLUTE);
                            _DEV_MC[HLP].RBJoint_SetMoveJoint(QJ.HLP, t_target_hind, MOVE_ABSOLUTE);
                            _DEV_MC[HLK].RBJoint_SetMoveJoint(QJ.HLK, t_target_hind, MOVE_ABSOLUTE);
//                            _DEV_MC[FRR].RBJoint_SetMoveJoint(QJ.FRR, t_targ_hindet, MOVE_ABSOLUTE);
//                            _DEV_MC[FRP].RBJoint_SetMoveJoint(QJ.FRP, t_target, MOVE_ABSOLUTE);
//                            _DEV_MC[FRK].RBJoint_SetMoveJoint(QJ.FRK, t_target, MOVE_ABSOLUTE);
//                            _DEV_MC[FLR].RBJoint_SetMoveJoint(QJ.FLR, t_target, MOVE_ABSOLUTE);
//                            _DEV_MC[FLP].RBJoint_SetMoveJoint(QJ.FLP, t_target, MOVE_ABSOLUTE);
//                            _DEV_MC[FLK].RBJoint_SetMoveJoint(QJ.FLK, t_target, MOVE_ABSOLUTE);

                            usleep(t_target_hind * 1e3);
                        }




                        Quad.isReadyDone = true;

                        FILE_LOG(logSUCCESS) << "BACK FLIP SIMULATION Done.";
                    }

                }
                else{
                    FILE_LOG(logWARNING) << "Please READYPOS First.";
                }

                break;
            }
            case MOTION_JUMP:
            {
                sharedData->COMMAND.USER_COMMAND = NO_ACT;
                QuadJoint QJ;
                double t_target;

                if(Quad.isReadyDone)
                {
                    Quad.isReadyDone = false;
                    FILE_LOG(logINFO) << "HUBODOG5_JUMP";

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
                        QJ.HRP = 90;
                        QJ.HLP = 90;
                        QJ.HRK = -130;
                        QJ.HLK = -130;

                        //front
                        QJ.FRP = 90;
                        QJ.FLP = 90;
                        QJ.FRK = -130;
                        QJ.FLK = -130;

                        t_target = 1000;

                        _DEV_MC[FRP].RBJoint_SetMoveJoint(QJ.FRP, t_target, MOVE_ABSOLUTE);
                        _DEV_MC[FRK].RBJoint_SetMoveJoint(QJ.FRK, t_target, MOVE_ABSOLUTE);
                        _DEV_MC[FLP].RBJoint_SetMoveJoint(QJ.FLP, t_target, MOVE_ABSOLUTE);
                        _DEV_MC[FLK].RBJoint_SetMoveJoint(QJ.FLK, t_target, MOVE_ABSOLUTE);
                        _DEV_MC[HRP].RBJoint_SetMoveJoint(QJ.HRP, t_target, MOVE_ABSOLUTE);
                        _DEV_MC[HRK].RBJoint_SetMoveJoint(QJ.HRK, t_target, MOVE_ABSOLUTE);
                        _DEV_MC[HLP].RBJoint_SetMoveJoint(QJ.HLP, t_target, MOVE_ABSOLUTE);
                        _DEV_MC[HLK].RBJoint_SetMoveJoint(QJ.HLK, t_target, MOVE_ABSOLUTE);
                        usleep(t_target * 1e3);

                        // 2. front push
                        // hind
                        QJ.HRP = 80;
                        QJ.HLP = 80;
                        QJ.HRK = -130;
                        QJ.HLK = -130;

                        // front
                        QJ.FRP = (70);
                        QJ.FLP = (70);
                        QJ.FRK = -90;
                        QJ.FLK = -90;

                        t_target = (100);

                        _DEV_MC[FRP].RBJoint_SetMoveJoint(QJ.FRP, t_target, MOVE_ABSOLUTE);
                        _DEV_MC[FRK].RBJoint_SetMoveJoint(QJ.FRK, t_target, MOVE_ABSOLUTE);
                        _DEV_MC[FLP].RBJoint_SetMoveJoint(QJ.FLP, t_target, MOVE_ABSOLUTE);
                        _DEV_MC[FLK].RBJoint_SetMoveJoint(QJ.FLK, t_target, MOVE_ABSOLUTE);
                        _DEV_MC[HRP].RBJoint_SetMoveJoint(QJ.HRP, t_target, MOVE_ABSOLUTE);
                        _DEV_MC[HRK].RBJoint_SetMoveJoint(QJ.HRK, t_target, MOVE_ABSOLUTE);
                        _DEV_MC[HLP].RBJoint_SetMoveJoint(QJ.HLP, t_target, MOVE_ABSOLUTE);
                        _DEV_MC[HLK].RBJoint_SetMoveJoint(QJ.HLK, t_target, MOVE_ABSOLUTE);
                        usleep(t_target * 1e3);

                        // 3. front fold ready - hind rotation
                        // hind
                        QJ.HRP = 190;
                        QJ.HLP = 190;
                        QJ.HRK = -130;
                        QJ.HLK = -130;

                        // front
                        QJ.FRP = 90;
                        QJ.FLP = 90;
                        QJ.FRK = -120;
                        QJ.FLK = -120;

                        t_target = (300);

                        _DEV_MC[HRP].RBJoint_SetMoveJoint(QJ.HRP, t_target, MOVE_ABSOLUTE);
                        _DEV_MC[HRK].RBJoint_SetMoveJoint(QJ.HRK, t_target, MOVE_ABSOLUTE);
                        _DEV_MC[HLP].RBJoint_SetMoveJoint(QJ.HLP, t_target, MOVE_ABSOLUTE);
                        _DEV_MC[HLK].RBJoint_SetMoveJoint(QJ.HLK, t_target, MOVE_ABSOLUTE);
                        _DEV_MC[FRP].RBJoint_SetMoveJoint(QJ.FRP, t_target, MOVE_ABSOLUTE);
                        _DEV_MC[FRK].RBJoint_SetMoveJoint(QJ.FRK, t_target, MOVE_ABSOLUTE);
                        _DEV_MC[FLP].RBJoint_SetMoveJoint(QJ.FLP, t_target, MOVE_ABSOLUTE);
                        _DEV_MC[FLK].RBJoint_SetMoveJoint(QJ.FLK, t_target, MOVE_ABSOLUTE);
                        usleep(t_target * 1e3);

                        // 4. hind push and front fold
                        // hind
                        QJ.HRP = 135;
                        QJ.HLP = 135;
                        QJ.HRK = -30;
                        QJ.HLK = -30;

                        // front
                        QJ.FRP = 85;
                        QJ.FLP = 85;
                        QJ.FRK = -130;
                        QJ.FLK = -130;

                        t_target = (200);

                        _DEV_MC[FRP].RBJoint_SetMoveJoint(QJ.FRP, t_target, MOVE_ABSOLUTE);
                        _DEV_MC[FRK].RBJoint_SetMoveJoint(QJ.FRK, t_target, MOVE_ABSOLUTE);
                        _DEV_MC[FLP].RBJoint_SetMoveJoint(QJ.FLP, t_target, MOVE_ABSOLUTE);
                        _DEV_MC[FLK].RBJoint_SetMoveJoint(QJ.FLK, t_target, MOVE_ABSOLUTE);
                        _DEV_MC[HRP].RBJoint_SetMoveJoint(QJ.HRP, t_target, MOVE_ABSOLUTE);
                        _DEV_MC[HRK].RBJoint_SetMoveJoint(QJ.HRK, t_target, MOVE_ABSOLUTE);
                        _DEV_MC[HLP].RBJoint_SetMoveJoint(QJ.HLP, t_target, MOVE_ABSOLUTE);
                        _DEV_MC[HLK].RBJoint_SetMoveJoint(QJ.HLK, t_target, MOVE_ABSOLUTE);
                        usleep(t_target * 1e3);

                        usleep(100 * 1e3);

                        // 5. return to walk ready
                        // hind
                        QJ.HRP = 60;
                        QJ.HLP = 60;
                        QJ.HRK = -110;
                        QJ.HLK = -110;

                        // front
                        QJ.FRP = 60;
                        QJ.FLP = 60;
                        QJ.FRK = -110;
                        QJ.FLK = -110;

                        t_target = 300;

                        _DEV_MC[FRP].RBJoint_SetMoveJoint(QJ.FRP, t_target, MOVE_ABSOLUTE);
                        _DEV_MC[FRK].RBJoint_SetMoveJoint(QJ.FRK, t_target, MOVE_ABSOLUTE);
                        _DEV_MC[FLP].RBJoint_SetMoveJoint(QJ.FLP, t_target, MOVE_ABSOLUTE);
                        _DEV_MC[FLK].RBJoint_SetMoveJoint(QJ.FLK, t_target, MOVE_ABSOLUTE);
                        _DEV_MC[HRP].RBJoint_SetMoveJoint(QJ.HRP, t_target, MOVE_ABSOLUTE);
                        _DEV_MC[HRK].RBJoint_SetMoveJoint(QJ.HRK, t_target, MOVE_ABSOLUTE);
                        _DEV_MC[HLP].RBJoint_SetMoveJoint(QJ.HLP, t_target, MOVE_ABSOLUTE);
                        _DEV_MC[HLK].RBJoint_SetMoveJoint(QJ.HLK, t_target, MOVE_ABSOLUTE);
                        usleep(t_target * 1e3);

                        FILE_LOG(logSUCCESS) << "JUMP SIMULATION Done.";
                    }
                    else {
                        // 1. down
                        // hind
                        QJ.HRP = 90;
                        QJ.HLP = 90;
                        QJ.HRK = -130 - (QJ.HRP-30)/11;
                        QJ.HLK = -130 - (QJ.HLP-30)/11;

                        //front
                        QJ.FRP = 90;
                        QJ.FLP = 90;
                        QJ.FRK = -130 - (QJ.FRP-30)/11;
                        QJ.FLK = -130 - (QJ.FLP-30)/11;

                        t_target = 1000;

                        _DEV_MC[FRP].RBJoint_SetMoveJoint(QJ.FRP, t_target, MOVE_ABSOLUTE);
                        _DEV_MC[FRK].RBJoint_SetMoveJoint(QJ.FRK, t_target, MOVE_ABSOLUTE);
                        _DEV_MC[FLP].RBJoint_SetMoveJoint(QJ.FLP, t_target, MOVE_ABSOLUTE);
                        _DEV_MC[FLK].RBJoint_SetMoveJoint(QJ.FLK, t_target, MOVE_ABSOLUTE);
                        _DEV_MC[HRP].RBJoint_SetMoveJoint(QJ.HRP, t_target, MOVE_ABSOLUTE);
                        _DEV_MC[HRK].RBJoint_SetMoveJoint(QJ.HRK, t_target, MOVE_ABSOLUTE);
                        _DEV_MC[HLP].RBJoint_SetMoveJoint(QJ.HLP, t_target, MOVE_ABSOLUTE);
                        _DEV_MC[HLK].RBJoint_SetMoveJoint(QJ.HLK, t_target, MOVE_ABSOLUTE);
                        usleep(t_target * 1e3);

                        // 2. front push
                        // hind
                        QJ.HRP = 80;
                        QJ.HLP = 80;
                        QJ.HRK = -130 - (QJ.HRP-30)/11;
                        QJ.HLK = -130 - (QJ.HLP-30)/11;

                        // front
                        QJ.FRP = (70+3);
                        QJ.FLP = (70+3);
                        QJ.FRK = -90 - (QJ.FRP-30)/11;
                        QJ.FLK = -90 - (QJ.FLP-30)/11;

                        t_target = (100);

                        _DEV_MC[FRP].RBJoint_SetMoveJoint(QJ.FRP, t_target, MOVE_ABSOLUTE);
                        _DEV_MC[FRK].RBJoint_SetMoveJoint(QJ.FRK, t_target, MOVE_ABSOLUTE);
                        _DEV_MC[FLP].RBJoint_SetMoveJoint(QJ.FLP, t_target, MOVE_ABSOLUTE);
                        _DEV_MC[FLK].RBJoint_SetMoveJoint(QJ.FLK, t_target, MOVE_ABSOLUTE);
                        _DEV_MC[HRP].RBJoint_SetMoveJoint(QJ.HRP, t_target, MOVE_ABSOLUTE);
                        _DEV_MC[HRK].RBJoint_SetMoveJoint(QJ.HRK, t_target, MOVE_ABSOLUTE);
                        _DEV_MC[HLP].RBJoint_SetMoveJoint(QJ.HLP, t_target, MOVE_ABSOLUTE);
                        _DEV_MC[HLK].RBJoint_SetMoveJoint(QJ.HLK, t_target, MOVE_ABSOLUTE);
                        usleep(t_target * 1e3);

                        // 3. front fold ready - hind rotation
                        // hind
                        QJ.HRP = (190-20);
                        QJ.HLP = (190-20);
                        QJ.HRK = -130 - (QJ.HRP-30)/11;
                        QJ.HLK = -130 - (QJ.HLP-30)/11;

                        // front
                        QJ.FRP = 90;
                        QJ.FLP = 90;
                        QJ.FRK = -120 - (QJ.FRP-30)/11;
                        QJ.FLK = -120 - (QJ.FLP-30)/11;

                        t_target = (350);

                        _DEV_MC[HRP].RBJoint_SetMoveJoint(QJ.HRP, t_target, MOVE_ABSOLUTE);
                        _DEV_MC[HRK].RBJoint_SetMoveJoint(QJ.HRK, t_target, MOVE_ABSOLUTE);
                        _DEV_MC[HLP].RBJoint_SetMoveJoint(QJ.HLP, t_target, MOVE_ABSOLUTE);
                        _DEV_MC[HLK].RBJoint_SetMoveJoint(QJ.HLK, t_target, MOVE_ABSOLUTE);
                        _DEV_MC[FRP].RBJoint_SetMoveJoint(QJ.FRP, t_target, MOVE_ABSOLUTE);
                        _DEV_MC[FRK].RBJoint_SetMoveJoint(QJ.FRK, t_target, MOVE_ABSOLUTE);
                        _DEV_MC[FLP].RBJoint_SetMoveJoint(QJ.FLP, t_target, MOVE_ABSOLUTE);
                        _DEV_MC[FLK].RBJoint_SetMoveJoint(QJ.FLK, t_target, MOVE_ABSOLUTE);
                        usleep(t_target * 1e3);

                        // 4. hind push and front fold
                        // hind
                        QJ.HRP = (135-10);
                        QJ.HLP = (135-10);
                        QJ.HRK = -30 - (QJ.HRP-30)/11;
                        QJ.HLK = -30 - (QJ.HLP-30)/11;

                        // front
                        QJ.FRP = 85;
                        QJ.FLP = 85;
                        QJ.FRK = -130 - (QJ.FRP-30)/11;
                        QJ.FLK = -130 - (QJ.FLP-30)/11;

                        t_target = (130);

                        _DEV_MC[FRP].RBJoint_SetMoveJoint(QJ.FRP, t_target, MOVE_ABSOLUTE);
                        _DEV_MC[FRK].RBJoint_SetMoveJoint(QJ.FRK, t_target, MOVE_ABSOLUTE);
                        _DEV_MC[FLP].RBJoint_SetMoveJoint(QJ.FLP, t_target, MOVE_ABSOLUTE);
                        _DEV_MC[FLK].RBJoint_SetMoveJoint(QJ.FLK, t_target, MOVE_ABSOLUTE);
                        _DEV_MC[HRP].RBJoint_SetMoveJoint(QJ.HRP, t_target, MOVE_ABSOLUTE);
                        _DEV_MC[HRK].RBJoint_SetMoveJoint(QJ.HRK, t_target, MOVE_ABSOLUTE);
                        _DEV_MC[HLP].RBJoint_SetMoveJoint(QJ.HLP, t_target, MOVE_ABSOLUTE);
                        _DEV_MC[HLK].RBJoint_SetMoveJoint(QJ.HLK, t_target, MOVE_ABSOLUTE);
                        usleep(t_target * 1e3);

                        usleep(100 * 1e3);

                        // 5. return to walk ready

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


                        // hind
                        QJ.HRP = 60;
                        QJ.HLP = 60;
                        QJ.HRK = -110 - (QJ.HRP-30)/11;
                        QJ.HLK = -110 - (QJ.HLP-30)/11;

                        // front
                        QJ.FRP = 60;
                        QJ.FLP = 60;
                        QJ.FRK = -110 - (QJ.FRP-30)/11;
                        QJ.FLK = -110 - (QJ.FLP-30)/11;

                        t_target = 300;

                        _DEV_MC[FRP].RBJoint_SetMoveJoint(QJ.FRP, t_target, MOVE_ABSOLUTE);
                        _DEV_MC[FRK].RBJoint_SetMoveJoint(QJ.FRK, t_target, MOVE_ABSOLUTE);
                        _DEV_MC[FLP].RBJoint_SetMoveJoint(QJ.FLP, t_target, MOVE_ABSOLUTE);
                        _DEV_MC[FLK].RBJoint_SetMoveJoint(QJ.FLK, t_target, MOVE_ABSOLUTE);
                        _DEV_MC[HRP].RBJoint_SetMoveJoint(QJ.HRP, t_target, MOVE_ABSOLUTE);
                        _DEV_MC[HRK].RBJoint_SetMoveJoint(QJ.HRK, t_target, MOVE_ABSOLUTE);
                        _DEV_MC[HLP].RBJoint_SetMoveJoint(QJ.HLP, t_target, MOVE_ABSOLUTE);
                        _DEV_MC[HLK].RBJoint_SetMoveJoint(QJ.HLK, t_target, MOVE_ABSOLUTE);
                        usleep(t_target * 1e3);

                        FILE_LOG(logSUCCESS) << "JUMP REAL Done.";
                    }

                }
                break;
            }
            case MOTION_FALLRECOVER:
            {
                sharedData->COMMAND.USER_COMMAND = NO_ACT;
                FILE_LOG(logINFO) << "HUBODOG5_MOTION_FALLRECOVER";

                QuadJoint QJ;

                // INC GAIN
                for(int i=0;i<NO_OF_JOINTS;i++)
                {
                    MY_CONTROL_FB_GAIN[i] = 10;
                    MY_CONTROL_FF_GAIN[i] = 0;
                }
                usleep(2 * 1e3);

                if(sharedData->REF_Enabled == true) {
                    for(int i=50; i<200; i++) {
                        for(int j=0; j<12; j++) {_DEV_MC[j].RBFOC_SetGain_POS(i,0,600, 1);}

                        if(i < 50) usleep(10 * 1e3);
                        else if(i < 100) usleep(10 * 1e3);
                        else usleep(5 * 1e3);

                    }
                    for(int j=0; j<12; j++) {_DEV_MC[j].RBFOC_SetGain_POS(200,0,600, 0);}
                }
                usleep(100 * 1e3);

                // 1-1 rotate leg
                QJ.HRR = 0;
                QJ.HLR = 0;
                QJ.FRR = 0;
                QJ.FLR = 0;

                QJ.HRP = 65-180;//-10;
                QJ.HLP = 65-180;//-10;
                QJ.HRK = -130 - (QJ.HRP)/11;//+50;
                QJ.HLK = -130 - (QJ.HLP)/11;//+50;

                // front
                QJ.FRP = 65-180;//-10;
                QJ.FLP = 65-180;//-10;
                QJ.FRK = -130 - (QJ.FRP)/11;//+50;
                QJ.FLK = -130 - (QJ.FLP)/11;//+50;

                double t_target = 1500;

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

                //1-2 stand

                QJ.HRP = 55-180;//-10;
                QJ.HLP = 55-180;//-10;
                QJ.HRK = -100 - (QJ.HRP)/11;//+50;
                QJ.HLK = -100 - (QJ.HLP)/11;//+50;

                // front
                QJ.FRP = 55-180;//-10;
                QJ.FLP = 55-180;//-10;
                QJ.FRK = -100 - (QJ.FRP)/11;//+50;
                QJ.FLK = -100 - (QJ.FLP)/11;//+50;

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

                // GAIN INC
                if(sharedData->REF_Enabled == true) {
                    for(int i=0; i<100; i++) {
                        for(int j=0; j<12; j++) {_DEV_MC[j].RBFOC_SetGain_POS(i+200,(int)(i/5),600, 1);}

                        if(i < 50) usleep((70-i) * 1e3);
                        else if(i < 100) usleep(2 * 1e3);
                        else usleep(5 * 1e3);

                    }
                    for(int j=0; j<12; j++) {_DEV_MC[j].RBFOC_SetGain_POS(300,20,600, 0);}
                }


                // 2-1
                // hind
                QJ.HRP = QJ.HLP = 35 -180.0;
                QJ.HRK = QJ.HLK = -100 - (QJ.HRP/11.0);

                //front
                QJ.FRP = QJ.FLP = 35 -180.0;
                QJ.FRK = QJ.FLK = -100 - (QJ.FRP/11.0);

                t_target = 600;

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

                // 2-2 push
                // hind
                QJ.HRP = QJ.HLP = 25 -180.0;
                QJ.HRK = QJ.HLK = -50 - (QJ.HRP/11.0);

                // front
                QJ.FRP = QJ.FLP = 45 -180.0;
                QJ.FRK = QJ.FLK = -100 - (QJ.FRP/11.0);

                t_target = 90;

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

                // 2-3 front and hind rotation
                // hind walkready
                QJ.HRP = QJ.HLP = 180 -180.0;
                QJ.HRK = QJ.HLK = -140 - (QJ.HRP/11.0);
                double t_target_hind = 350;

                // front
                QJ.FRP = QJ.FLP = 55+180 -180.0;
                QJ.FRK = QJ.FLK = -100 - (QJ.FRP/11.0);
                t_target = 700+50;
                _DEV_MC[HRR].RBJoint_SetMoveJoint(QJ.HRR, t_target_hind, MOVE_ABSOLUTE);
                _DEV_MC[HRP].RBJoint_SetMoveJoint(QJ.HRP, t_target_hind, MOVE_ABSOLUTE);
                _DEV_MC[HRK].RBJoint_SetMoveJoint(QJ.HRK, t_target_hind, MOVE_ABSOLUTE);
                _DEV_MC[HLR].RBJoint_SetMoveJoint(QJ.HLR, t_target_hind, MOVE_ABSOLUTE);
                _DEV_MC[HLP].RBJoint_SetMoveJoint(QJ.HLP, t_target_hind, MOVE_ABSOLUTE);
                _DEV_MC[HLK].RBJoint_SetMoveJoint(QJ.HLK, t_target_hind, MOVE_ABSOLUTE);
                _DEV_MC[FRR].RBJoint_SetMoveJoint(QJ.FRR, t_target, MOVE_ABSOLUTE);
                _DEV_MC[FRP].RBJoint_SetMoveJoint(QJ.FRP, t_target, MOVE_ABSOLUTE);
                _DEV_MC[FRK].RBJoint_SetMoveJoint(QJ.FRK, t_target, MOVE_ABSOLUTE);
                _DEV_MC[FLR].RBJoint_SetMoveJoint(QJ.FLR, t_target, MOVE_ABSOLUTE);
                _DEV_MC[FLP].RBJoint_SetMoveJoint(QJ.FLP, t_target, MOVE_ABSOLUTE);
                _DEV_MC[FLK].RBJoint_SetMoveJoint(QJ.FLK, t_target, MOVE_ABSOLUTE);
                usleep(t_target_hind * 1e3);


                if(sharedData->REF_Enabled == true) {
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
                }

                // 2-4 return to walk ready

                QJ.HRP = QJ.HLP = 55+180 -180.0;
                QJ.HRK = QJ.HLK = -100 - (QJ.HRP/11.0);

                t_target_hind = 100;

                _DEV_MC[HRR].RBJoint_SetMoveJoint(QJ.HRR, t_target_hind, MOVE_ABSOLUTE);
                _DEV_MC[HRP].RBJoint_SetMoveJoint(QJ.HRP, t_target_hind, MOVE_ABSOLUTE);
                _DEV_MC[HRK].RBJoint_SetMoveJoint(QJ.HRK, t_target_hind, MOVE_ABSOLUTE);
                _DEV_MC[HLR].RBJoint_SetMoveJoint(QJ.HLR, t_target_hind, MOVE_ABSOLUTE);
                _DEV_MC[HLP].RBJoint_SetMoveJoint(QJ.HLP, t_target_hind, MOVE_ABSOLUTE);
                _DEV_MC[HLK].RBJoint_SetMoveJoint(QJ.HLK, t_target_hind, MOVE_ABSOLUTE);

                usleep(t_target * 1e3);

                // GAIN INC
                if(sharedData->REF_Enabled == true) {
                    for(int i=50; i<200; i++) {
                        for(int j=0; j<12; j++) {_DEV_MC[j].RBFOC_SetGain_POS(i,0,600, 1);}

                        if(i < 50) usleep(10 * 1e3);
                        else if(i < 100) usleep(10 * 1e3);
                        else usleep(5 * 1e3);

                    }
                    for(int j=0; j<12; j++) {_DEV_MC[j].RBFOC_SetGain_POS(200,0,600, 0);}
                }
                usleep(100 * 1e3);

                QJ.HRR = _DEV_MC[HRR].homeJointReady;
                QJ.HRP = _DEV_MC[HRP].homeJointReady;
                QJ.HRK = _DEV_MC[HRK].homeJointReady;
                QJ.HLR = _DEV_MC[HLR].homeJointReady;
                QJ.HLP = _DEV_MC[HLP].homeJointReady;
                QJ.HLK = _DEV_MC[HLK].homeJointReady;
                QJ.FRR = _DEV_MC[FRR].homeJointReady;
                QJ.FRP = _DEV_MC[FRP].homeJointReady;
                QJ.FRK = _DEV_MC[FRK].homeJointReady;
                QJ.FLR = _DEV_MC[FLR].homeJointReady;
                QJ.FLP = _DEV_MC[FLP].homeJointReady;
                QJ.FLK = _DEV_MC[FLK].homeJointReady;

                _DEV_MC[HRR].RBJoint_SetMoveJoint(QJ.HRR, ready_ms, MOVE_ABSOLUTE);
                _DEV_MC[HRP].RBJoint_SetMoveJoint(QJ.HRP, ready_ms, MOVE_ABSOLUTE);
                _DEV_MC[HRK].RBJoint_SetMoveJoint(QJ.HRK, ready_ms, MOVE_ABSOLUTE);
                _DEV_MC[HLR].RBJoint_SetMoveJoint(QJ.HLR, ready_ms, MOVE_ABSOLUTE);
                _DEV_MC[HLP].RBJoint_SetMoveJoint(QJ.HLP, ready_ms, MOVE_ABSOLUTE);
                _DEV_MC[HLK].RBJoint_SetMoveJoint(QJ.HLK, ready_ms, MOVE_ABSOLUTE);
                _DEV_MC[FRR].RBJoint_SetMoveJoint(QJ.FRR, ready_ms, MOVE_ABSOLUTE);
                _DEV_MC[FRP].RBJoint_SetMoveJoint(QJ.FRP, ready_ms, MOVE_ABSOLUTE);
                _DEV_MC[FRK].RBJoint_SetMoveJoint(QJ.FRK, ready_ms, MOVE_ABSOLUTE);
                _DEV_MC[FLR].RBJoint_SetMoveJoint(QJ.FLR, ready_ms, MOVE_ABSOLUTE);
                _DEV_MC[FLP].RBJoint_SetMoveJoint(QJ.FLP, ready_ms, MOVE_ABSOLUTE);
                _DEV_MC[FLK].RBJoint_SetMoveJoint(QJ.FLK, ready_ms, MOVE_ABSOLUTE);

                usleep(ready_ms * 1e3);

                FILE_LOG(logSUCCESS) << "READYPOS Done.";
                Quad.isReadyDone = true;
                sharedData->isFalled = false;

                break;
            }
            case INCGAIN2:
            {
                for(int i=0;i<NO_OF_JOINTS;i++)
                {
                    MY_CONTROL_FB_GAIN[i] = 10;
                    //MY_CONTROL_FF_GAIN[i] = 0;
                }
                usleep(2 * 1e3);

                if(sharedData->REF_Enabled == true) {
                    for(int i=50; i<200; i++) {
                        for(int j=0; j<12; j++) {_DEV_MC[j].RBFOC_SetGain_POS(i,0,600, 1);}

                        if(i < 50) usleep(10 * 1e3);
                        else if(i < 100) usleep(20 * 1e3);
                        else usleep(5 * 1e3);

                    }
                }
                for(int j=0; j<12; j++) {_DEV_MC[j].RBFOC_SetGain_POS(200,0,600, 0);}

                FILE_LOG(logSUCCESS) << "GAIN INC 2 Done.";
                sharedData->COMMAND.USER_COMMAND = NO_ACT;
                break;

//                REF_modi_cnt = 0;
//                QJ_ref[HRR] = Quad.JointRef.HRR;
//                QJ_ref[HRP] = Quad.JointRef.HRP;
//                QJ_ref[HRK] = Quad.JointRef.HRK;
//                QJ_ref[HLR] = Quad.JointRef.HLR;
//                QJ_ref[HLP] = Quad.JointRef.HLP;
//                QJ_ref[HLK] = Quad.JointRef.HLK;
//                QJ_ref[FRR] = Quad.JointRef.FRR;
//                QJ_ref[FRP] = Quad.JointRef.FRP;
//                QJ_ref[FRK] = Quad.JointRef.FRK;
//                QJ_ref[FLR] = Quad.JointRef.FLR;
//                QJ_ref[FLP] = Quad.JointRef.FLP;
//                QJ_ref[FLK] = Quad.JointRef.FLK;



//                sharedData->incGain_flag = true;
//                incgain2_flag = false;

//                FILE_LOG(logSUCCESS) << "Ref modi start.";
//                sharedData->COMMAND.USER_COMMAND = NO_ACT;
//                break;
            }
            case INCGAIN:
            {
                for(int i=0;i<NO_OF_JOINTS;i++)
                {
                    MY_CONTROL_FB_GAIN[i] = 10;
                    MY_CONTROL_FF_GAIN[i] = 0;
                }
                usleep(2 * 1e3);

                if(sharedData->REF_Enabled == true) {
                    for(int i=50; i<200; i++) {
                        for(int j=0; j<12; j++) {_DEV_MC[j].RBFOC_SetGain_POS(i,0,600, 1);}

                        if(i < 50) usleep(10 * 1e3);
                        else if(i < 100) usleep(20 * 1e3);
                        else usleep(5 * 1e3);

                    }
                }
                for(int j=0; j<12; j++) {_DEV_MC[j].RBFOC_SetGain_POS(200,0,600, 0);}

                FILE_LOG(logSUCCESS) << "GAIN INC Done.";
                sharedData->COMMAND.USER_COMMAND = NO_ACT;
                break;
            }
            case MOTOR_OFF:
            {
                FILE_LOG(logSUCCESS) << "HUBODOG5_MOTOR_OFF";
                for(int i=0;i<NO_OF_JOINTS;i++)
                {
                    _DEV_MC[i].RBJoint_EnableFeedbackControl(0,DISABLE);
                }
                if(Quad.isMotionEnabled)
                {
                    Quad.isMotionEnabled = false;
                }
                FILE_LOG(logSUCCESS) << "CREF ZERO";
                for(int i=0;i<NO_OF_JOINTS;i++)
                {
                    _DEV_MC[i].MoveJoints.RefAngleCurrent = _DEV_MC[i].CurrentPosition;
                    _DEV_MC[i].CurrentReference_mA = 0;
                }
                sharedData->COMMAND.USER_COMMAND = NO_ACT;

                break;
            }
            case MOTION_START:
            {
                sharedData->COMMAND.USER_COMMAND = NO_ACT;
                if(Quad.isReadyDone)
                {
                    FILE_LOG(logSUCCESS) << "HUBODOG5_MOTION_START";
                    if(sharedData->isSimulation)
                    {
                        FILE_LOG(logSUCCESS) << "Simulation Mode ON.";
                        Quad.isSimulation = true;
                    }
                    else{
                        FILE_LOG(logSUCCESS) << "Experiment Mode ON.";
                        Quad.isSimulation = false;
                    }

                    if(Quad.isMotionEnabled == false)
                    {
                        //imu_zero();
                    }

                    Quad.initialize();
                    Quad.motion_start();
                }
                else{
                    FILE_LOG(logWARNING) << "Please READYPOS First.";
                }

                break;
            }
            case MOTION_STOP:
            {
                sharedData->COMMAND.USER_COMMAND = NO_ACT;
                FILE_LOG(logINFO) << "HUBODOG5_MOTION_STOP";

                for(int i=0;i<NO_OF_JOINTS;i++)
                {
                    MY_CONTROL_FB_GAIN[i] = 10;
                    MY_CONTROL_FF_GAIN[i] = 0;
                }
                usleep(2 * 1e3);

                if(sharedData->REF_Enabled == true) {
                    for(int i=50; i<200; i++) {
                        for(int j=0; j<12; j++) {_DEV_MC[j].RBFOC_SetGain_POS(i,0,600, 1);}

                        if(i < 50) usleep(10 * 1e3);
                        else if(i < 100) usleep(20 * 1e3);
                        else usleep(5 * 1e3);

                    }
                }
                for(int j=0; j<12; j++) {_DEV_MC[j].RBFOC_SetGain_POS(200,0,600, 0);}

//                for(int i=0;i<NO_OF_JOINTS;i++)
//                {
//                    MY_CONTROL_FB_GAIN[i] = 10;
//                }
//                for(int i=0; i<10; i++) {
//                    for(int j=0; j<12; j++) {
//                        MY_CONTROL_FF_GAIN[j]--;
//                    }
//                    usleep(50 * 1e3);
//                }

//                //qwer

//                for(int i=0;i<NO_OF_JOINTS;i++)
//                {
//                    MY_CONTROL_FB_GAIN[i] = 10;
//                    MY_CONTROL_FF_GAIN[i] = 0;
//                }

                QuadJoint QJ;
                QJ.HRR = _DEV_MC[HRR].homeJointReady;
                QJ.HRP = _DEV_MC[HRP].homeJointReady;
                QJ.HRK = _DEV_MC[HRK].homeJointReady;
                QJ.HLR = _DEV_MC[HLR].homeJointReady;
                QJ.HLP = _DEV_MC[HLP].homeJointReady;
                QJ.HLK = _DEV_MC[HLK].homeJointReady;
                QJ.FRR = _DEV_MC[FRR].homeJointReady;
                QJ.FRP = _DEV_MC[FRP].homeJointReady;
                QJ.FRK = _DEV_MC[FRK].homeJointReady;
                QJ.FLR = _DEV_MC[FLR].homeJointReady;
                QJ.FLP = _DEV_MC[FLP].homeJointReady;
                QJ.FLK = _DEV_MC[FLK].homeJointReady;

                _DEV_MC[HRR].RBJoint_SetMoveJoint(QJ.HRR, ready_ms, MOVE_ABSOLUTE);
                _DEV_MC[HRP].RBJoint_SetMoveJoint(QJ.HRP, ready_ms, MOVE_ABSOLUTE);
                _DEV_MC[HRK].RBJoint_SetMoveJoint(QJ.HRK, ready_ms, MOVE_ABSOLUTE);
                _DEV_MC[HLR].RBJoint_SetMoveJoint(QJ.HLR, ready_ms, MOVE_ABSOLUTE);
                _DEV_MC[HLP].RBJoint_SetMoveJoint(QJ.HLP, ready_ms, MOVE_ABSOLUTE);
                _DEV_MC[HLK].RBJoint_SetMoveJoint(QJ.HLK, ready_ms, MOVE_ABSOLUTE);
                _DEV_MC[FRR].RBJoint_SetMoveJoint(QJ.FRR, ready_ms, MOVE_ABSOLUTE);
                _DEV_MC[FRP].RBJoint_SetMoveJoint(QJ.FRP, ready_ms, MOVE_ABSOLUTE);
                _DEV_MC[FRK].RBJoint_SetMoveJoint(QJ.FRK, ready_ms, MOVE_ABSOLUTE);
                _DEV_MC[FLR].RBJoint_SetMoveJoint(QJ.FLR, ready_ms, MOVE_ABSOLUTE);
                _DEV_MC[FLP].RBJoint_SetMoveJoint(QJ.FLP, ready_ms, MOVE_ABSOLUTE);
                _DEV_MC[FLK].RBJoint_SetMoveJoint(QJ.FLK, ready_ms, MOVE_ABSOLUTE);

                usleep(ready_ms * 1e3);


                Quad.motion_stop();
                Quad.isReadyDone = true;
                sharedData->isStopping = false;
                sharedData->isFalled = false;

                FILE_LOG(logSUCCESS) << "MOTION STOP Done.";

                break;
            }
            case MOTION_FALLING:
            {
                sharedData->COMMAND.USER_COMMAND = NO_ACT;
                FILE_LOG(logINFO) << "HUBODOG5_MOTION_FALLING";

                for(int i=0;i<NO_OF_JOINTS;i++)
                {
                    MY_CONTROL_FB_GAIN[i] = 10;
                    MY_CONTROL_FF_GAIN[i] = 0;
                }

                for(int j=0; j<12; j++) {_DEV_MC[j].RBFOC_SetGain_POS(50,0,600, 1);}

                QuadJoint QJ;

                QJ.HRR = 0;
                QJ.HLR = 0;
                QJ.FRR = 0;
                QJ.FLR = 0;

                QJ.HRP = 65;//-10;
                QJ.HLP = 65;//-10;
                QJ.HRK = -130 - (QJ.HRP)/11;//+50;
                QJ.HLK = -130 - (QJ.HLP)/11;//+50;

                // front
                QJ.FRP = 65;//-10;
                QJ.FLP = 65;//-10;
                QJ.FRK = -130 - (QJ.FRP)/11;//+50;
                QJ.FLK = -130 - (QJ.FLP)/11;//+50;

                double t_target = 200;

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

                sharedData->isFalled = true;

                break;
            }
            default:
                break;
            }
        }
       usleep(100*1000);//10Hz message dealing
    }
}


//MPC part
void thread_MPC()
{
    if(Quad.isMotionEnabled)
    {
        if(Quad.isMPCON)
        {
            //Quad.MPC_calc();
        }
    }
}


void main_controller(){
    // Main Controller

    // get joystick

    set_ref_data();
    get_sensor_data();

//    cout << "sensor get" << endl;

    if(Quad.isMotionEnabled)
    {
        Quad.state_estimator();

        if(Quad.isPosAdjust){
            Quad.JointRef = Quad.pos_adjust();
        }
        else if(Quad.task_idx == TrotTest){
            Quad.JointRef = Quad.task_trot_test();
        }
        else if(Quad.task_idx == HoppingTest){
            Quad.JointRef = Quad.task_hopping_test();
        }
        else if(Quad.task_idx == Flyingtrot){
            Quad.JointRef = Quad.task_flyingtrot();
        }

        SetAllCurrentRef(Quad.CurRef);
        SetAllJointRef(Quad.JointRef);

    }

    for(int i=0; i<NO_OF_JOINTS; i++) // GUI info
    {
        sharedData->PCON[i] = MY_CONTROL_FB_GAIN[i];
        sharedData->CCON[i] = MY_CONTROL_FF_GAIN[i];
    }


//    JointRef = sensor.Encoder.pos;
//    cout << JointRef.HRK << endl;
//    SetAllJointRef(JointRef);
//    usleep(1e6);

}
