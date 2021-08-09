#include "RBMotorController.h"
#include "JointInformation.h"

RBMotorController::RBMotorController()
{
    MoveJoints.MoveFlag = false;
    CurrentStatus.B[0] = CurrentStatus.B[1] = CurrentStatus.B[2] = 0;
}

void RBMotorController::RBJoint_SetMoveJoint(float angle, float timeMs, int mode){
    if(MoveJoints.MoveFlag == true){
//        FILE_LOG(logWARNING) << "RBJoint_SetMoveJoint working now..[BNO: " << BOARD_ID << "]";
        return;
    }
    MoveJoints.Profile = 0;
    MoveJoints.RefAngleInitial = MoveJoints.RefAngleCurrent;
    if(mode == 0){  // abs
        MoveJoints.RefAngleToGo = angle;
        MoveJoints.RefAngleDelta = MoveJoints.RefAngleToGo - MoveJoints.RefAngleInitial;
    }else{          // rel
        MoveJoints.RefAngleToGo = MoveJoints.RefAngleInitial + angle;
        MoveJoints.RefAngleDelta = angle;
    }

    MoveJoints.GoalTimeCount = (ulong)(timeMs/(double)RT_TIMER_PERIOD_MS);
    MoveJoints.CurrentTimeCount = 0;
    MoveJoints.MoveFlag = true;
}

void RBMotorController::RBJoint_SetMoveJointSTrapi(float angle, float speed, int mode){
    if(MoveJoints.MoveFlag == true){
        FILE_LOG(logWARNING) << "RBJoint_SetMoveJointSTrapi working now..[BNO: " << BOARD_ID << "]";
        return;
    }
    MoveJoints.Profile = 1;
    MoveJoints.RefAngleInitial = MoveJoints.RefAngleCurrent;
    if(mode == 0){  // abs
        MoveJoints.RefAngleToGo = angle;
        MoveJoints.RefAngleDelta = MoveJoints.RefAngleToGo - MoveJoints.RefAngleInitial;
    }else{          // rel
        MoveJoints.RefAngleToGo = MoveJoints.RefAngleInitial + angle;
        MoveJoints.RefAngleDelta = angle;
    }

    MoveJoints.STP_Speed = fabs(speed);
    if(MoveJoints.STP_Speed * MoveJoints.STP_Speed / MoveJoints.STP_MaxAcc >= fabs(MoveJoints.RefAngleDelta)){
        MoveJoints.STP_Type = 1;
        MoveJoints.STP_acctime = sqrt(fabs(MoveJoints.RefAngleDelta)/MoveJoints.STP_MaxAcc);
        MoveJoints.STP_goaltime = MoveJoints.STP_acctime * 2;
        if(MoveJoints.STP_acctime < 0.005){
            MoveJoints.STP_AccTimeCnt = 1;
            MoveJoints.GoalTimeCount = 2;
        }else{
            MoveJoints.STP_AccTimeCnt = (uint)(1000.0 * MoveJoints.STP_acctime /(double)RT_TIMER_PERIOD_MS);
            MoveJoints.GoalTimeCount = (uint)(1000.0 * MoveJoints.STP_goaltime /(double)RT_TIMER_PERIOD_MS);
        }
        MoveJoints.acctime = MoveJoints.STP_AccTimeCnt*(double)RT_TIMER_PERIOD_MS / 1000.0;
        MoveJoints.STP_acc = fabs(MoveJoints.RefAngleDelta) / (MoveJoints.acctime*MoveJoints.acctime);
    }else{
        MoveJoints.STP_Type = 0;
        MoveJoints.STP_acctime = MoveJoints.STP_Speed / MoveJoints.STP_MaxAcc;
        MoveJoints.STP_goaltime = MoveJoints.STP_acctime * 2 + (fabs(MoveJoints.RefAngleDelta) - MoveJoints.STP_Speed*MoveJoints.STP_acctime) / MoveJoints.STP_Speed;
        MoveJoints.STP_AccTimeCnt = (uint)(1000.0 * MoveJoints.STP_acctime /(double)RT_TIMER_PERIOD_MS);
        MoveJoints.GoalTimeCount = (uint)(1000.0 * MoveJoints.STP_goaltime /(double)RT_TIMER_PERIOD_MS);
        MoveJoints.acctime = MoveJoints.STP_AccTimeCnt*(double)RT_TIMER_PERIOD_MS / 1000.0;
        MoveJoints.goaltime = MoveJoints.GoalTimeCount*(double)RT_TIMER_PERIOD_MS / 1000.0;
        if(MoveJoints.acctime < 2){
            MoveJoints.STP_acc = MoveJoints.STP_MaxAcc;
        }else{
            MoveJoints.STP_acc = fabs(MoveJoints.RefAngleDelta) / (MoveJoints.goaltime*MoveJoints.acctime - MoveJoints.acctime*MoveJoints.acctime);
        }
        FILE_LOG(logWARNING) << MoveJoints.RefAngleDelta << ", " << MoveJoints.goaltime << ", " << MoveJoints.acctime;
    }
    MoveJoints.CurrentTimeCount = 0;
    MoveJoints.MoveFlag = true;
}

float moving_angle = 0.0;
void RBMotorController::RBJoint_MoveJoint(){

    if(MoveJoints.MoveFlag){
        MoveJoints.CurrentTimeCount++;

        if(MoveJoints.GoalTimeCount <= MoveJoints.CurrentTimeCount)
        {
            MoveJoints.GoalTimeCount = MoveJoints.CurrentTimeCount = 0;
            MoveJoints.RefAngleCurrent = MoveJoints.RefAngleToGo;
//            std::cout << "MoveFlag Off" << std::endl;
            MoveJoints.MoveFlag = false;
        }
        else
        {
            MoveJoints.RefAngleCurrent = MoveJoints.RefAngleInitial + MoveJoints.RefAngleDelta*0.5*
                (1.0f-cos(RBCORE_PI/(double)MoveJoints.GoalTimeCount*(double)MoveJoints.CurrentTimeCount));
//            MoveJoints.RefAngleCurrent = MoveJoints.RefAngleToGo;
        }
    }
    else{
//        MoveJoints.RefAngleCurrent = MoveJoints.RefAngleToGo;
    }
}

void RBMotorController::RBMC_AddCANMailBox(){
    canHandler->RBCAN_AddMailBox(ID_RCV_ENC);
    canHandler->RBCAN_AddMailBox(ID_RCV_INFO);
    canHandler->RBCAN_AddMailBox(ID_RCV_PARA);
    canHandler->RBCAN_AddMailBox(ID_RCV_STAT);
}

void RBMotorController::RBBoard_GetDBData(DB_MC db){
    BOARD_ID        = db.BOARD_ID;
    BOARD_TYPE      = db.BOARD_TYPE;
    MOTOR_CHANNEL   = db.MOTOR_CHANNEL;
    CAN_CHANNEL     = db.CAN_CHANNEL;
    ID_SEND_REF     = db.ID_SEND_REF;
    ID_SEND_REF2     = db.ID_SEND_REF2;
    ID_RCV_ENC      = db.ID_RCV_ENC;
    ID_RCV_STAT     = db.ID_RCV_STAT;
    ID_RCV_INFO     = db.ID_RCV_INFO;
    ID_RCV_PARA     = db.ID_RCV_PARA;
    ID_SEND_GENERAL = db.ID_SEND_GENERAL;
    TOTAL_CHANNEL   = BOARD_TYPE;

    for(int i=0; i<MOTOR_CHANNEL; i++){
        PPR = db.JOINTS[i].PPR;
    }
}

void RBMotorController::RBBoard_SetCANIDs(int bno, int ch, int can_ch){
    BOARD_ID = bno;
    BOARD_TYPE = ch;
    MOTOR_CHANNEL = ch;
    CAN_CHANNEL = can_ch;
//    ID_SEND_REF = 0x10+bno;
//    ID_RCV_ENC = 0x60+bno;
//    ID_RCV_STAT = 0x150+bno;
//    ID_RCV_INFO = 0x190+bno;
//    ID_RCV_PARA = 0x1C0+bno;
//    ID_SEND_GENERAL = 0x210+bno;

    ID_SEND_GENERAL = 0x10+bno;
    ID_RCV_INFO = 0x20+bno;
    ID_RCV_PARA = 0x20+bno;
    ID_RCV_STAT = 0x30+bno;
    ID_SEND_REF = 0x40+bno;
    ID_SEND_REF2 = 0x90+bno; // uiuk added
    ID_RCV_ENC = 0x50+bno;

    TOTAL_CHANNEL = ch;
}



void RBMotorController::RBBoard_ReferenceOutEnable(bool _refEnable){
    ReferenceOutEnable = _refEnable;
}

int RBMotorController::RBBoard_CANCheck(int _canr){
//    RBCAN_MB mb;
//    mb.channel = CAN_CHANNEL;
//    mb.data[0] = 1;
//    mb.dlc = 1;
////    mb.id = COMMAND_CANID;
//    mb.id = ID_SEND_GENERAL;

//    if(canHandler->RBCAN_WriteData(mb) == true){
//        usleep(50*1000);
//        mb.channel = CAN_CHANNEL;
//        mb.id = ID_RCV_INFO;
//        canHandler->RBCAN_ReadData(&mb);
//        if(mb.status != RBCAN_NODATA){
//            std::cout << ">>> RMMC: Board(" << BOARD_ID  << ") is \033[32minitialized.\033[0m[CAN ch " << CAN_CHANNEL << "]" << std::endl;
//            ConnectionStatus = true;
//            mb.status = RBCAN_NODATA;
//            return true;
//        }else{
//            std::cout << ">>> RMMC: Board(" << BOARD_ID << ") is \033[31mfailed \033[0mto initialize.[CAN ch " << CAN_CHANNEL << "]" << std::endl;
//            ConnectionStatus = false;
//            return false;
//        }
//    }
//    else return false;


    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 0x01;
    mb.dlc = 8;
    unsigned short over_current = 19*1000;//mA
    unsigned short over_current_time = 100;//ms
    unsigned short current_limit = 17*1000;//mA
    mb.data[2] = over_current & 0xFF;
    mb.data[3] = (over_current>>8)& 0xFF;
    mb.data[4] = over_current_time & 0xFF;
    mb.data[5] = (over_current_time>>8)& 0xFF;
    mb.data[6] = current_limit & 0xFF;
    mb.data[7] = (current_limit>>8)& 0xFF;
    mb.id = ID_SEND_GENERAL;
    int ret = canHandler->RBCAN_WriteData(mb);

    usleep(30*1000);
    return ret;
}

int RBMotorController::RBBoard_RequestStatus(void){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 0x02;
    mb.dlc = 1;
    mb.id = ID_SEND_GENERAL;

    return canHandler->RBCAN_WriteDataDirectly(mb);
}

int RBMotorController::RBBoard_GetStatus(void){
    RBCAN_MB mb;
    unsigned int ret = false;

    mb.channel = CAN_CHANNEL;
    mb.id = ID_RCV_STAT;
    canHandler->RBCAN_ReadData(&mb);
    if(mb.status != RBCAN_NODATA)    {
//        for(int i=0; i<2; i++){
            CurrentStatus.B[0] = mb.data[0];
            CurrentStatus.B[1] = mb.data[1];
            CurrentStatus.B[2] = mb.data[2];
//        }
        ret = true;
        mb.status = RBCAN_NODATA;
    }
    return ret;
}

int RBMotorController::RBBoard_LoadDefaultValue(void){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 0xFA;
    mb.dlc = 1;
    mb.id = ID_SEND_GENERAL;

    return canHandler->RBCAN_WriteData(mb);
}

int RBMotorController::RBBoard_SetBoardNumber(int _newbno, int _canr){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 0xF0;
    mb.data[1] = _newbno;
    mb.data[2] = _canr;
    mb.dlc = 3;
    mb.id = ID_SEND_GENERAL;

    return canHandler->RBCAN_WriteData(mb);
}

int RBMotorController::RBBoard_SetControlTime(float period_ms){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;

    int tmp = period_ms*20.0; // 20khz low-level resolution
    mb.data[0] = 0xF1;
    mb.data[1] = tmp;
    mb.dlc = 2;
    mb.id = ID_SEND_GENERAL;

    return canHandler->RBCAN_WriteData(mb);
}

int RBMotorController::RBBoard_RequestEncoder(int mode){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 0x03;
    mb.data[1] = mode;      // 1-continuous, 0-oneshot
    mb.dlc = 2;
    mb.id = ID_SEND_GENERAL;

    return canHandler->RBCAN_WriteData(mb);
}

int RBMotorController::RBBoard_RequestCurrent(void){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 0x04;
    mb.dlc = 1;
    mb.id = ID_SEND_GENERAL;

    return canHandler->RBCAN_WriteData(mb);
}

int RBMotorController::RBJoint_ResetEncoder(int ch){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 0x06;
    mb.data[1] = ch;
    mb.dlc = 2;
    mb.id = ID_SEND_GENERAL;

    return canHandler->RBCAN_WriteData(mb);
}

int RBMotorController::RBJoint_EnableFETDriver(int ch, int enable){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 0x0B;
    mb.data[1] = ch;
    mb.data[2] = enable;        // 1-enable, 0-disable
    mb.dlc = 3;
    mb.id = ID_SEND_GENERAL;

    return canHandler->RBCAN_WriteData(mb);
}

int RBMotorController::RBJoint_EnableFeedbackControl(int ch, int enable, int mode){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 0x0E;
    mb.data[1] = ch;
    mb.data[2] = enable;    // 1-enable, 0-disable
    mb.data[3] = mode;
    mb.dlc = 4;
    mb.id = ID_SEND_GENERAL;

    return canHandler->RBCAN_WriteDataDirectly(mb);
}

int RBMotorController::RBJoint_FindHome(int ch){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 0x11;
    mb.data[1] = ch;
    mb.dlc = 2;
    mb.id = ID_SEND_GENERAL;

    return canHandler->RBCAN_WriteData(mb);
}

int RBMotorController::RBJoint_EnableFrictionCompensation(int ch, int enable){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 0xB1;
    mb.data[1] = ch;
    mb.data[2] = enable;    // 1-enable, 0-disable
    mb.dlc = 3;
    mb.id = ID_SEND_GENERAL;

    return canHandler->RBCAN_WriteData(mb);
}

int RBMotorController::RBJoint_SetMaxDuty(int ch, int duty){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 0xB2;
    mb.data[1] = ch;
    mb.data[2] = duty;  // 1~100 (%)
    mb.dlc = 3;
    mb.id = ID_SEND_GENERAL;

    return canHandler->RBCAN_WriteData(mb);
}

int RBMotorController::RBJoint_SetPositionCommandMode(int _mode, int _ch){
    // MODE ==> 0: absolute position reference
    //      ==> 1: incremental position reference
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 0x14;							// command
    mb.data[1] = _ch;
    mb.data[2] = _mode;
    mb.dlc = 3;
    mb.id = ID_SEND_GENERAL;

    return canHandler->RBCAN_WriteData(mb);
}

int RBMotorController::RBJoint_ClearErrorFlag(int ch){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 0x75;
    mb.data[1] = ch;
    mb.dlc = 2;
    mb.id = ID_SEND_GENERAL;

    return canHandler->RBCAN_WriteData(mb);
}



int RBMotorController::RBBoard_RequestTemperature(){
//    RBCAN_MB mb;
//    mb.channel = CAN_CHANNEL;
//    mb.dlc = 1;
//    mb.data[0] = 0x04;							// command
//    mb.id = ID_SEND_GENERAL;

    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.dlc = 1;
    mb.data[0] = 0xA1;							// command
    mb.id = ID_SEND_GENERAL;

    return canHandler->RBCAN_WriteDataDirectly(mb);
}

int RBMotorController::RBBoard_ReadTemperature(void){
    RBCAN_MB mb;
    int tempInt;
    int mTemp1;//, mTemp2;//findwarning

    mb.channel = CAN_CHANNEL;
    mb.id = ID_RCV_PARA;
    canHandler->RBCAN_ReadData(&mb);
    if(mb.status != RBCAN_NODATA){
        tempInt = (int)((mb.data[1]<<8) | (mb.data[0]));
        mTemp1 = (int)((mb.data[3]<<8) | (mb.data[2]));
//        mTemp2 = (int)((mb.data[5]<<8) | (mb.data[4]));//findwarning

        BoardTemperature = ((float)tempInt)/10.0f;
        MotorTemperature = ((float)mTemp1)/10.0f;
        //MotorTemperature = ((float)mTemp2)/10.0f;

        mb.status = RBCAN_NODATA;
        return true;
    }else{
        return false;
    }
}

int RBMotorController::RBBoard_PWMCommand2ch(int mode1, short duty1, int mode2, short duty2){
    // mode ================
    // 0: not applied for channel x
    // 1: open-loop PWM in % duty
    // 2: open-loop PWM in 0.1% resolution duty
    // 3: feed-forward open-loop PWM in 0.1% resolution duty
    // 4: feed-forward current-mapped PWM(mA)
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 0x0D;
    mb.data[1] = mode1;
    mb.data[2] = (unsigned char)(duty1 & 0x00FF);
    mb.data[3] = (unsigned char)((duty1>>8) & 0x00FF);

    mb.data[4] = mode2;
    mb.data[5] = (unsigned char)(duty2 & 0x00FF);
    mb.data[6] = (unsigned char)((duty2>>8) & 0x00FF);
    mb.dlc = 7;
    mb.id = ID_SEND_GENERAL;

    return canHandler->RBCAN_WriteData(mb);
}

int RBMotorController::RBBoard_SetControlMode(int _mode){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 0x10;		// command
    mb.data[1] = _mode;	// control mode
    // _mode = 0x00 : position control mode
    // _mode = 0x01 : current control mode
    mb.dlc = 2;
    mb.id = ID_SEND_GENERAL;

    return canHandler->RBCAN_WriteData(mb);
}


int InitControlState = false;
int InitControlState_prev[12] = {false,};
int MY_CONTROL_MODE_SELECTION;
extern int MY_CONTROL_FB_GAIN[12];
extern int MY_CONTROL_FF_GAIN[12];
//int RBMotorController::RBBoard_SendReference2ch(int ref1, int current_limit_T){
//    RBCAN_MB mb;
//    mb.channel = CAN_CHANNEL;

//    int send_ref = ref1&0x007FFFFF;
//    if(ref1<0){
//        send_ref |= 0x00800000;
//    }

//    // position
//    mb.data[0] = send_ref & 0xFF;
//    mb.data[1] = (send_ref>>8) & 0xFF;
//    mb.data[2] = (send_ref>>16) & 0xFF;

//    // current
//    mb.data[3] = (unsigned char)(CurrentReference_mA&0xFF);
//    mb.data[4] = (CurrentReference_mA>>8)&0xFF;
////    mb.data[5] = (unsigned char)(enc_request_T & 0xFF);
//    mb.data[5] = (unsigned char)(current_limit_T & 0b01111111);
//    if(MY_CONTROL_MODE_SELECTION == 1){
//        mb.data[5] = mb.data[5] | 0b10000000;
//    }
//    mb.data[6] = (MY_CONTROL_FB_GAIN[BOARD_ID]<<4 & 0xF0) | (MY_CONTROL_FF_GAIN[BOARD_ID] & 0x0F);
//    mb.dlc = 7;
////    mb.dlc = 6;
//    mb.id = ID_SEND_REF;


//    return canHandler->RBCAN_WriteDataDirectly(mb);
//}

int RBMotorController::RBBoard_SendReference2ch(int ref1, int enc_request_T){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;

    int send_ref = ref1&0x007FFFFF;
    if(ref1<0){
        send_ref |= 0x00800000;
    }

    // position
    mb.data[0] = send_ref & 0xFF;
    mb.data[1] = (send_ref>>8) & 0xFF;
    mb.data[2] = (send_ref>>16) & 0xFF;

    // current
    mb.data[3] = (unsigned char)(CurrentReference_mA&0xFF);
    mb.data[4] = (CurrentReference_mA>>8)&0xFF;
    mb.data[5] = (unsigned char)(enc_request_T & 0xFF);
    if(MY_CONTROL_MODE_SELECTION == 1){
        mb.data[5] = mb.data[5] | 0b10000000;
    }
    mb.data[6] = (MY_CONTROL_FB_GAIN[BOARD_ID]<<4 & 0xF0) | (MY_CONTROL_FF_GAIN[BOARD_ID] & 0x0F);
    mb.dlc = 7;
    mb.id = ID_SEND_REF;


    return canHandler->RBCAN_WriteDataDirectly(mb);
}

int RBMotorController::RBBoard_SendReference(int enc_request_time){
    int ref;
    if(ReferenceOutEnable == false) return false;

    RefPosOld = RefPos;
    RefPos = Reference;
    ref = (int)(RefPos*PPR);
    return RBBoard_SendReference2ch(ref, enc_request_time);
}

int RBMotorController::RBBoard_SetErrorBound(int _ierror, int _berror, int _teeror){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 0xF3;						// command
    mb.data[1] = (_ierror & 0xFF);			// maximum input difference error (new ref - old ref)
    mb.data[2] = ((_ierror>>8) & (0xFF));	// maximum input difference error (new ref - old ref)
    mb.data[3] = (_berror & 0xFF);			// maximum error (ref - encoder pos)
    mb.data[4] = ((_berror>>8) & (0xFF));	// maximum error (ref - encoder pos)
    mb.data[5] = (_teeror & 0xFF);			// maximum temerature warning
    mb.data[6] = ((_teeror>>8) & (0xFF));	// maximum temerature warning
    mb.dlc = 7;
    mb.id = ID_SEND_GENERAL;

    return canHandler->RBCAN_WriteData(mb);
}

int RBMotorController::RBBoard_RequestParameter(int _para){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 0x24;			// command
    mb.data[1] = _para;		// parameter request
    // OF = 0 : channel 0, 1 and 2
    // OF = 1 : channel 3 and 4
    // _para = channel*6 + OF + 1 : Motor position control gains and encoder resolution
    // _para = channel*6 + OF + 2 : Deadzone, home search direction, home search mode and home search limit
    // _para = channel*6 + OF + 3 : Home offset and lower position limit
    // _para = channel*6 + OF + 4 : Upper limit position, maximum acceleration, maximum velocity and maximum PWM
    // _para = channel*6 + OF + 5 : Current limit and motor current control gains
    // _para = channel*6 + OF + 6 : motor current control gains
    // _para = 20 : Board number, CAN comunication rate, board type and maximun acceleration for home limit search
    // _para = 21 : Maximun velocity to limit switch, maximun velocity to offset position, duration for JAM detection and duration for saturation detection
    // _para = 22 : PWM duty for saturation, PWM duty for JAM and error bound values
    mb.dlc = 2;
    mb.id = ID_SEND_GENERAL;

    return canHandler->RBCAN_WriteData(mb);
}


#define CMD_SOLENOID_CONTROL        0xC0
#define CMD_CHECK_UVW               0xC1
#define CMD_DQ_ALIGN                0xC2
#define CMD_INIT_CONTROL            0xC3
#define CMD_FRICTION_COMPENSATION   0xC4
#define CMD_MU_DISK_OFFSET          0xC5
#define CMD_HOME_ZERO_OFFSET        0xC6
#define CMD_ENCODER_REQUESTTIME     0xC8


#define CMD_FOC_CURRENT_CONTROL     0xD0
#define CMD_FOC_AUTO_TUNE_CURRENT   0xD1
#define CMD_FOC_GAIN                0xD2
#define CMD_FOC_OPENLOOP_CONTROL    0xD3
#define CMD_FOC_AUTO_TUNE_POSITION  0xD4
#define CMD_POS_GAIN                0xD5
#define CMD_CURRENT_NULLING         0xD6
#define CMD_MEASURE_L               0xE0
#define CMD_SINE_WAVE_TEST          0xE1
#define CMD_PWM_DEADTIME            0xE5

#define CMD_MU150_CMD               0xEE

int RBMotorController::RB_InitDummy(){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 0x02;
    mb.dlc = 1;
    mb.id = ID_SEND_GENERAL;
    return canHandler->RBCAN_WriteData(mb);
}

int RBMotorController::RB_InitControl(unsigned int sol_duty){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = CMD_INIT_CONTROL;
    mb.data[1] = sol_duty;
    mb.dlc = 2;
    mb.id = ID_SEND_GENERAL;
    return canHandler->RBCAN_WriteData(mb);
}

int RBMotorController::RB_SolenoidControl(int mode){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = CMD_SOLENOID_CONTROL;
    mb.data[1] = mode;  // 0 off    1 hard      2 weak
    mb.dlc = 2;
    mb.id = ID_SEND_GENERAL;
    return canHandler->RBCAN_WriteData(mb);
}

int RBMotorController::RB_CheckUVW(){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = CMD_CHECK_UVW;
    mb.dlc = 1;
    mb.id = ID_SEND_GENERAL;
    return canHandler->RBCAN_WriteData(mb);
}

int RBMotorController::RB_DQAlign(){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = CMD_DQ_ALIGN;
    mb.dlc = 1;
    mb.id = ID_SEND_GENERAL;
    return canHandler->RBCAN_WriteData(mb);
}

int RBMotorController::RB_CurrentNulling(){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = CMD_CURRENT_NULLING;
    mb.data[1] = 0;
    mb.dlc = 2;
    mb.id = ID_SEND_GENERAL;
    return canHandler->RBCAN_WriteData(mb);
}

int RBMotorController::RB_ControlFrictionCompensation(int start_stop){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = CMD_FRICTION_COMPENSATION;
    if(start_stop)
        mb.data[1] = 2;
    else
        mb.data[1] = 3;
    mb.dlc = 2;
    mb.id = ID_SEND_GENERAL;
    return canHandler->RBCAN_WriteData(mb);
}


int RBMotorController::RB_SetFrictionParam(int low_vel, int high_vel, int amp){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = CMD_FRICTION_COMPENSATION;
    mb.data[1] = 0;
    mb.data[2] = low_vel & 0xFF;
    mb.data[3] = (low_vel>>8) & 0xFF;
    mb.data[4] = high_vel & 0xFF;
    mb.data[5] = (high_vel>>8) & 0xFF;
    mb.data[6] = amp & 0xFF;
    mb.data[7] = (amp>>8) & 0xFF;
    mb.dlc = 8;
    mb.id = ID_SEND_GENERAL;
    return canHandler->RBCAN_WriteData(mb);
}

int RBMotorController::RB_SetFrictionParam_1(int low_vel, int high_vel){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = CMD_FRICTION_COMPENSATION;
    mb.data[1] = 0;
    mb.data[2] = 0;
    mb.data[3] = low_vel & 0xFF;
    mb.data[4] = (low_vel>>8) & 0xFF;
    mb.data[5] = high_vel & 0xFF;
    mb.data[6] = (high_vel>>8) & 0xFF;
    mb.dlc = 7;
    mb.id = ID_SEND_GENERAL;
    return canHandler->RBCAN_WriteData(mb);
}

int RBMotorController::RB_SetFrictionParam_2(int amp, int dead_amp){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = CMD_FRICTION_COMPENSATION;
    mb.data[1] = 0;
    mb.data[2] = 1;
    mb.data[3] = amp & 0xFF;
    mb.data[4] = (amp>>8) & 0xFF;
    mb.data[5] = dead_amp & 0xFF;
    mb.data[6] = (dead_amp>>8) & 0xFF;
    mb.dlc = 7;
    mb.id = ID_SEND_GENERAL;
    return canHandler->RBCAN_WriteData(mb);
}

int RBMotorController::RB_RequestFrictionParam(){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = CMD_FRICTION_COMPENSATION;
    mb.data[1] = 1;
    mb.dlc = 2;
    mb.id = ID_SEND_GENERAL;

    if(canHandler->RBCAN_WriteData(mb) == true){
        usleep(30*1000);
        mb.channel = CAN_CHANNEL;
        mb.id = ID_RCV_PARA;
        canHandler->RBCAN_ReadData(&mb);
        if(mb.status != RBCAN_NODATA){
            if(mb.data[0] == CMD_FRICTION_COMPENSATION){
                FrictionParam_LowVel = (int)((short)(mb.data[1] | (mb.data[2]<<8)));
                FrictionParam_HighVel = (int)((short)(mb.data[3] | (mb.data[4]<<8)));
                FrictionParam_Amp = (int)((short)(mb.data[5] | (mb.data[6]<<8)));
            }
            mb.status = RBCAN_NODATA;
            return true;
        }else{
            return false;
        }
    }
    else return false;
}

//int RBMotorController::RBHome_RequestIndexPulse(){
//    RBCAN_MB mb;
//    mb.channel = CAN_CHANNEL;
//    mb.data[0] = CMD_CARIBRATE_ABS_ENC;
//    mb.data[1] = 1;
//    mb.dlc = 2;
//    mb.id = ID_SEND_GENERAL;

//    if(canHandler->RBCAN_WriteData(mb) == true){
//        usleep(30*1000);
//        mb.channel = CAN_CHANNEL;
//        mb.id = ID_RCV_PARA;
//        canHandler->RBCAN_ReadData(&mb);
//        if(mb.status != RBCAN_NODATA){
//            if(mb.data[0] == CMD_CARIBRATE_ABS_ENC){
//                Home_IndexPulse = (int)((mb.data[1] | (mb.data[2]<<8) | (mb.data[3]<<16) | (mb.data[4]<<24)));
//                Home_AbsPulse = (short)(mb.data[5] | (mb.data[6]<<8));
//            }
//            mb.status = RBCAN_NODATA;
//            return true;
//        }else{
//            return false;
//        }
//    }
//    else return false;
//}

int RBMotorController::RBHome_RequestHomeZeroOffset(){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = CMD_HOME_ZERO_OFFSET;
    mb.data[1] = 1;
    mb.dlc = 2;
    mb.id = ID_SEND_GENERAL;

    if(canHandler->RBCAN_WriteData(mb) == true){
        usleep(30*1000);
        mb.channel = CAN_CHANNEL;
        mb.id = ID_RCV_PARA;
        canHandler->RBCAN_ReadData(&mb);
        if(mb.status != RBCAN_NODATA){
            if(mb.data[0] == CMD_HOME_ZERO_OFFSET){
                Home_ZeroOffset = (int)((mb.data[1] | (mb.data[2]<<8) | (mb.data[3]<<16) | (mb.data[4]<<24)));
            }
            mb.status = RBCAN_NODATA;
            return true;
        }else{
            return false;
        }
    }
    else return false;
}

int RBMotorController::RBHome_SetHomeZeroOffset(int offset, int now_zero){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = CMD_HOME_ZERO_OFFSET;
    mb.data[1] = 0;
    mb.data[2] = offset & 0xFF;
    mb.data[3] = (offset>>8) & 0xFF;
    mb.data[4] = (offset>>16) & 0xFF;
    mb.data[5] = (offset>>24) & 0xFF;
    mb.data[6] = now_zero;
    mb.dlc = 7;
    mb.id = ID_SEND_GENERAL;
    return canHandler->RBCAN_WriteData(mb);
}





int RBMotorController::RBFOC_RequestAutoTuneParams(){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = CMD_FOC_AUTO_TUNE_CURRENT;
    mb.data[1] = 1;
    mb.dlc = 2;
    mb.id = ID_SEND_GENERAL;

    if(canHandler->RBCAN_WriteData(mb) == true){
        usleep(30*1000);
        mb.channel = CAN_CHANNEL;
        mb.id = ID_RCV_PARA;
        canHandler->RBCAN_ReadData(&mb);
        if(mb.status != RBCAN_NODATA){
            if(mb.data[0] == CMD_FOC_AUTO_TUNE_CURRENT){
                AutoTune_CurOffset = (int)((short)(mb.data[1] | (mb.data[2]<<8)));
                AutoTune_InputVq = (int)((short)(mb.data[3] | (mb.data[4]<<8)));
            }
            mb.status = RBCAN_NODATA;
            return true;
        }else{
            return false;
        }
    }
    else return false;
}

int RBMotorController::RBFOC_SetAutoTuneParams(int at_curoffset, int at_inputvq){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = CMD_FOC_AUTO_TUNE_CURRENT;
    mb.data[1] = 0;
    mb.data[2] = at_curoffset & 0xFF;
    mb.data[3] = (at_curoffset>>8) & 0xFF;
    mb.data[4] = at_inputvq & 0xFF;
    mb.data[5] = (at_inputvq>>8) & 0xFF;
    mb.dlc = 6;
    mb.id = ID_SEND_GENERAL;
    return canHandler->RBCAN_WriteData(mb);
}

int RBMotorController::RBFOC_StartAutoTune(){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = CMD_FOC_AUTO_TUNE_CURRENT;
    mb.data[1] = 2;
    mb.dlc = 2;
    mb.id = ID_SEND_GENERAL;
    return canHandler->RBCAN_WriteData(mb);
}


int RBMotorController::RBFOC_RequestGain(){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = CMD_FOC_GAIN;
    mb.data[1] = 1;
    mb.dlc = 2;
    mb.id = ID_SEND_GENERAL;

    if(canHandler->RBCAN_WriteData(mb) == true){
        usleep(30*1000);
        mb.channel = CAN_CHANNEL;
        mb.id = ID_RCV_PARA;
        canHandler->RBCAN_ReadData(&mb);
        if(mb.status != RBCAN_NODATA){
            if(mb.data[0] == CMD_FOC_GAIN){
                FOC_PGain = (int)((short)(mb.data[1] | (mb.data[2]<<8)));
                FOC_IGain = (int)((short)(mb.data[3] | (mb.data[4]<<8)));
            }
            mb.status = RBCAN_NODATA;
            return true;
        }else{
            return false;
        }
    }
    else return false;
}

int RBMotorController::RBFOC_SetGain(int pgain, int igain, int mode){
    RBCAN_MB mb;

    if(mode == 0) {
        mb.channel = CAN_CHANNEL;
        mb.data[0] = CMD_FOC_GAIN;
        mb.data[1] = 0;
        mb.data[2] = pgain & 0xFF;
        mb.data[3] = (pgain>>8) & 0xFF;
        mb.data[4] = igain & 0xFF;
        mb.data[5] = (igain>>8) & 0xFF;
        mb.dlc = 6;
        mb.id = ID_SEND_GENERAL;
        canHandler->RBCAN_WriteData(mb);

        usleep(100*1000);

        mb.channel = CAN_CHANNEL;
        mb.data[0] = CMD_FOC_GAIN;
        mb.data[1] = 1;
        mb.dlc = 2;
        mb.id = ID_SEND_GENERAL;
        canHandler->RBCAN_WriteData(mb);
    }
    else if(mode == 1) {
        mb.channel = CAN_CHANNEL;
        mb.data[0] = CMD_FOC_GAIN;
        mb.data[1] = 0;
        mb.data[2] = pgain & 0xFF;
        mb.data[3] = (pgain>>8) & 0xFF;
        mb.data[4] = igain & 0xFF;
        mb.data[5] = (igain>>8) & 0xFF;
        mb.dlc = 6;
        mb.id = ID_SEND_GENERAL;
        canHandler->RBCAN_WriteData(mb);
    }
    else if(mode == 2) {
        mb.channel = CAN_CHANNEL;
        mb.data[0] = CMD_FOC_GAIN;
        mb.data[1] = 1;
        mb.dlc = 2;
        mb.id = ID_SEND_GENERAL;
        canHandler->RBCAN_WriteData(mb);
    }
    return false;
}

int RBMotorController::RBFOC_CurrentControl(int onoff){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = CMD_FOC_CURRENT_CONTROL;
    mb.data[1] = onoff;
    mb.dlc = 2;
    mb.id = ID_SEND_GENERAL;
    return canHandler->RBCAN_WriteDataDirectly(mb);
}

int RBMotorController::RBFOC_OpenLoopControl(int onoff, int inputV){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = CMD_FOC_OPENLOOP_CONTROL;
    mb.data[1] = onoff;
    mb.data[2] = inputV & 0xFF;
    mb.data[3] = (inputV>>8) & 0xFF;
    mb.dlc = 4;
    mb.id = ID_SEND_GENERAL;
    return canHandler->RBCAN_WriteData(mb);
}





int RBMotorController::RBFOC_RequestAutoTuneParams_POS(){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = CMD_FOC_AUTO_TUNE_POSITION;
    mb.data[1] = 1;
    mb.dlc = 2;
    mb.id = ID_SEND_GENERAL;

    if(canHandler->RBCAN_WriteData(mb) == true){
        usleep(30*1000);
        mb.channel = CAN_CHANNEL;
        mb.id = ID_RCV_PARA;
        canHandler->RBCAN_ReadData(&mb);
        if(mb.status != RBCAN_NODATA){
            if(mb.data[0] == CMD_FOC_AUTO_TUNE_POSITION){
                AutoTune_PosOffset = (int)((short)(mb.data[1] | (mb.data[2]<<8)));
                AutoTune_InputCurrent = (int)((short)(mb.data[3] | (mb.data[4]<<8)));
            }
            mb.status = RBCAN_NODATA;
            return true;
        }else{
            return false;
        }
    }
    else return false;
}

int RBMotorController::RBFOC_SetAutoTuneParams_POS(int at_posoffset, int at_inputcurrent){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = CMD_FOC_AUTO_TUNE_POSITION;
    mb.data[1] = 0;
    mb.data[2] = at_posoffset & 0xFF;
    mb.data[3] = (at_posoffset>>8) & 0xFF;
    mb.data[4] = at_inputcurrent & 0xFF;
    mb.data[5] = (at_inputcurrent>>8) & 0xFF;
    mb.dlc = 6;
    mb.id = ID_SEND_GENERAL;
    return canHandler->RBCAN_WriteData(mb);
}

int RBMotorController::RBFOC_StartAutoTune_POS(){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = CMD_FOC_AUTO_TUNE_POSITION;
    mb.data[1] = 2;
    mb.dlc = 2;
    mb.id = ID_SEND_GENERAL;
    return canHandler->RBCAN_WriteData(mb);
}


int RBMotorController::RBFOC_RequestGain_POS(){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = CMD_POS_GAIN;
    mb.data[1] = 1;
    mb.dlc = 2;
    mb.id = ID_SEND_GENERAL;

    if(canHandler->RBCAN_WriteData(mb) == true){
        usleep(30*1000);
        mb.channel = CAN_CHANNEL;
        mb.id = ID_RCV_PARA;
        canHandler->RBCAN_ReadData(&mb);
        if(mb.status != RBCAN_NODATA){
            if(mb.data[0] == CMD_POS_GAIN){
                POS_PGain = (int)((short)(mb.data[1] | (mb.data[2]<<8)));
                POS_IGain = (int)((short)(mb.data[3] | (mb.data[4]<<8)));
                POS_DGain = (int)((short)(mb.data[5] | (mb.data[6]<<8)));
            }
            mb.status = RBCAN_NODATA;
            return true;
        }else{
            return false;
        }
    }
    else return false;
}

int RBMotorController::RBFOC_SetGain_POS(int pgain, int igain, int dgain, int mode){
    RBCAN_MB mb;

    if(mode == 0) {
        mb.channel = CAN_CHANNEL;
        mb.data[0] = CMD_POS_GAIN;
        mb.data[1] = 0;
        mb.data[2] = pgain & 0xFF;
        mb.data[3] = (pgain>>8) & 0xFF;
        mb.data[4] = igain & 0xFF;
        mb.data[5] = (igain>>8) & 0xFF;
        mb.data[6] = dgain & 0xFF;
        mb.data[7] = (dgain>>8) & 0xFF;
        mb.dlc = 8;
        mb.id = ID_SEND_GENERAL;
        canHandler->RBCAN_WriteData(mb);

        usleep(10*1000);

        mb.channel = CAN_CHANNEL;
        mb.data[0] = CMD_POS_GAIN;
        mb.data[1] = 1;
        mb.dlc = 2;
        mb.id = ID_SEND_GENERAL;
        canHandler->RBCAN_WriteData(mb);
    }
    else if(mode == 1) {
        mb.channel = CAN_CHANNEL;
        mb.data[0] = CMD_POS_GAIN;
        mb.data[1] = 0;
        mb.data[2] = pgain & 0xFF;
        mb.data[3] = (pgain>>8) & 0xFF;
        mb.data[4] = igain & 0xFF;
        mb.data[5] = (igain>>8) & 0xFF;
        mb.data[6] = dgain & 0xFF;
        mb.data[7] = (dgain>>8) & 0xFF;
        mb.dlc = 8;
        mb.id = ID_SEND_GENERAL;
        canHandler->RBCAN_WriteData(mb);
    }
    else if(mode == 2) {
        mb.channel = CAN_CHANNEL;
        mb.data[0] = CMD_POS_GAIN;
        mb.data[1] = 1;
        mb.dlc = 2;
        mb.id = ID_SEND_GENERAL;
        canHandler->RBCAN_WriteData(mb);
    }
}


int RBMotorController::RBFOC_Measure_L(){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = CMD_MEASURE_L;
    mb.data[1] = 2;
    mb.dlc = 2;
    mb.id = ID_SEND_GENERAL;
    return canHandler->RBCAN_WriteData(mb);
}


int RBMotorController::RBFOC_SineWaveTest(int start_stop, int amp, int period){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = CMD_SINE_WAVE_TEST;
    mb.id = ID_SEND_GENERAL;

    mb.data[1] = start_stop;
    mb.data[2] = amp&0xFF;
    mb.data[3] = (amp>>8)&0xFF;
    mb.data[4] = (amp>>16)&0xFF;
    mb.data[5] = (amp>>24)&0xFF;
    mb.data[6] = period&0xFF;
    mb.data[7] = (period>>8)&0xFF;
    mb.dlc = 8;
    return canHandler->RBCAN_WriteData(mb);

}



int RBMotorController::RB_PWM_DeadTime(int deadtime_ns){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = CMD_PWM_DEADTIME;
    mb.id = ID_SEND_GENERAL;

    mb.data[1] = 0; // set
    mb.data[2] = deadtime_ns&0xFF;
    mb.data[3] = (deadtime_ns>>8)&0xFF;
    mb.dlc = 4;
    mb.id = ID_SEND_GENERAL;
    return canHandler->RBCAN_WriteData(mb);
}


int RBMotorController::RB_PVL_RESET(){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = CMD_MU150_CMD;
    mb.data[1] = 4;
    mb.dlc = 2;
    mb.id = ID_SEND_GENERAL;
    return canHandler->RBCAN_WriteData(mb);
}

int RBMotorController::RB_SAVE_MU_DISK_OFFSET(){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = CMD_MU_DISK_OFFSET;
    mb.data[1] = 0;
    mb.dlc = 2;
    mb.id = ID_SEND_GENERAL;
    return canHandler->RBCAN_WriteData(mb);
}

int RBMotorController::RB_SPI_OFF()
{
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 0xEE;
    mb.data[1] = 7;
    mb.data[2] = 1;
    mb.dlc = 3;
    mb.id = ID_SEND_GENERAL;
    return canHandler->RBCAN_WriteData(mb);
}

// uiuk added - encoder request time
int RBMotorController::RB_Encoder_RequestTime(int enc_request_time){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = CMD_ENCODER_REQUESTTIME;		// command
    mb.data[1] = enc_request_time;	// control mode
    mb.dlc = 2;
    mb.id = ID_SEND_GENERAL;

    return canHandler->RBCAN_WriteData(mb);
}
