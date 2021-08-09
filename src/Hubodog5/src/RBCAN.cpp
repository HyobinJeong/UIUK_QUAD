#include "RBCAN.h"
#include "RBMotion.h"
#include "RBMotorController.h"
#include "RBLANComm.h"

//#define CheapIMU

extern RBMotorController    _DEV_MC[MAX_MC];
extern RBLANComm   *lanHandler;
extern pRBCORE_SHM sharedData;

using namespace std;

extern int IS_NEW_CAN_DATA;

int CAN_COUNT_ERR_index = 0;

double Imu_yaw;

// --------------------------------------------------------------------------------------------- //
void RBCAN::RBCAN_ReadThread(void *_arg){
    RBCAN *rbCAN = (RBCAN *)_arg;

    unsigned int tcp_status = 0x00;
    int tcp_size = 0;
    unsigned char totalLANData[RX_DATA_SIZE];
    unsigned char tempPacketData[RX_DATA_SIZE];
    int connectCnt = 0;

    int index;

    while(rbCAN->isWorking == true){
        if(rbCAN->isSuspend == true){
            usleep(10);
            continue;
        }
        for(int i=0; i<rbCAN->chNum; i++){
            if(tcp_status == 0x00){     // if client was not connected
                if(rbCAN->LAN_CLIENT_FD == 0){
//                    rbCAN->CreateSocket("192.168.0.73", 1977);
                    rbCAN->CreateSocket("10.0.1.1", 1977);
                    FILE_LOG(logSUCCESS) << "CreateSocket";
                }
                if(rbCAN->Connect2Server()){
                    tcp_status = 0x01;
                    rbCAN->RXDataReadIndex = rbCAN->RXDataStoreIndex = 0;
                    rbCAN->ConnectionStatus = true;
                    FILE_LOG(logSUCCESS) << "Connect to Server Success..!!";
                }else{
                    //if(connectCnt%10 == 0)
//                    FILE_LOG(logERROR) << "Connect to Server Failed..!!";
                    connectCnt++;
                }
                usleep(1000*1000);
            }

            if(tcp_status == 0x01){     // if client was connected
                // tcp_size is bytes of received data.
                // If 0 was returned, it means the connect is closed
                // tcp_size = read(rbLAN->LAN_CLIENT_FD, &rbLAN->RXData[Owner->RXDataStoreIndex], RX_DATA_SIZE);
                tcp_size = read(rbCAN->LAN_CLIENT_FD, totalLANData, RX_DATA_SIZE);

                if(tcp_size == 0){
                    rbCAN->LanTCPClientClose();
                    rbCAN->LAN_CLIENT_FD = 0;//findwarning
                    tcp_status = 0x00;
                    rbCAN->RXDataReadIndex = rbCAN->RXDataStoreIndex = 0;
                    rbCAN->ConnectionStatus = false;
                    FILE_LOG(logERROR) << "Client is disconnected..!!";
                }else if(tcp_size > 0){
                    //FILE_LOG(logWARNING) << "TCP SIZE: " << tcp_size;


                    int start_pos = 0;
                    int end_pos = 0;
                    while(rbCAN->isWorking == true){
                        if(tcp_size-start_pos >= 4){
                            int packet_length = (short)(totalLANData[start_pos+1] | totalLANData[start_pos+2]<<8);
                            if(packet_length < 0){
                                cout << "RBLANComm:: packet length under zero" << endl;
                                start_pos = start_pos+1;
                            }else{
                                end_pos = start_pos + packet_length + 3 - 1;
            //                    cout << "start --------- " << start_pos << endl;
            //                    cout << "end ----------- " << end_pos << endl;

                                if(tcp_size-start_pos >= packet_length + 3){
                                    memcpy(tempPacketData, &totalLANData[start_pos], packet_length+3);

                                    if(tempPacketData[0] == 0x24 && tempPacketData[packet_length + 3 - 1] == 0x25){
                                        int data_type = tempPacketData[5];
                                        switch(data_type){
                                        case 0x00:
                                        {
                                            // CAN Data
                                            int num = (short)(tempPacketData[6] | (tempPacketData[7]<<8));

                                            for(int j=0; j<num; j++){
                                                unsigned char tempCANData[12];
                                                for(int k=0; k<12; k++){
                                                    tempCANData[k] = tempPacketData[8 + 12*j + k];
                                                }

                                                int ch_type = tempCANData[0];
                                                int id = (short)((tempCANData[1]) | tempCANData[2]<<8);
                                                int dlc = tempCANData[3];
                                                unsigned char data[8];
                                                memcpy(data, &tempCANData[4], dlc);

                                                if(ch_type == 0){
                                                    _IS_NEW_CAN_DATA = 1;
                                                }




                                                if(id >= 900 && id <= 911){
                                                    char msg[30];
                                                    sprintf(msg, "CPU %d RST", id-900);
//                                                    RCR_LOG(LOG_TYPE_FATAL, msg);
                                                }else if(id >= 950 && id <= 961){

                                                    if(tempCANData[4] == 0){
//                                                        char msg[30];
                                                        double del_gap = ((double)((int)(tempCANData[5] | (tempCANData[6] << 8) | (tempCANData[7] << 16) | (tempCANData[8] << 24))))/100.0;
//                                                        cout<<"CNT ERR"<<endl;
                                                        cout<<"CAN:"<<id-950<<"CNT E:"<<del_gap<<"ms"<<endl;
                                                        CAN_COUNT_ERR_index = 0;
//                                                        sprintf(msg, "CAN %d CNT ERR : %.1fms", id-950, del_gap);
//                                                        RCR_LOG(LOG_TYPE_ERROR, msg);
                                                    }else{

                                                    }

                                                }

                                                index = rbCAN->RBCAN_GetMailBoxIndex(id);

                                                if(index < rbCAN->canMBCounter){
                                                   rbCAN->canReadMB[index].id = id;
                                                   rbCAN->canReadMB[index].dlc = dlc;
                                                   memcpy(rbCAN->canReadMB[index].data, &tempCANData[4], dlc);
                                                   rbCAN->canReadMB[index].channel = i;
                                                   if(rbCAN->canReadMB[index].status == RBCAN_NEWDATA)
                                                       rbCAN->canReadMB[index].status = RBCAN_OVERWRITE;
                                                   else rbCAN->canReadMB[index].status = RBCAN_NEWDATA;
                                                }
                                            }
                                            break;
                                        }
                                        default:
                                            break;
                                        }
                                    }else{
                                        cout << "RBLANComm:: header footer not match" << endl;
                                    }
                                }else{
                                    cout << "RBLANComm:: size not match : " << tcp_size-start_pos-1 << ", " << end_pos << endl;
                                    break;
                                }
                                start_pos = end_pos+1;
                            }
                        }else{
                            break;
                        }
                    }
                }
            }
        }
        usleep(10);
    }
}
// --------------------------------------------------------------------------------------------- //
void *RBCAN::RBCAN_WriteThread(void *_arg){
    RBCAN *rbCAN = (RBCAN *)_arg;

//    int index = 0;//findwarning

    while(rbCAN->isWorking){
        if(rbCAN->isSuspend == true){
            usleep(3);
            continue;
        }
        if(rbCAN->canSendSuspend == true){
            if(rbCAN->canHeadIndex != rbCAN->canTailIndex){
//                index = rbCAN->canTailIndex;
//                FILE_LOG(logWARNING) << "Working";
//                rbCAN->canTailIndex = (rbCAN->canTailIndex + 1) % RBCAN_MAX_MB;
            }
        }
        usleep(5);
    }
    return NULL;
}
// --------------------------------------------------------------------------------------------- //


// --------------------------------------------------------------------------------------------- //
RBCAN::RBCAN(int _ChNum){
//    if(_ChNum > RBCAN_MAX_CAN_CHANNEL){
//        FILE_LOG(logERROR) << "Over the maximum CAN channel [" << RBCAN_MAX_CAN_CHANNEL << " ]";
//        chNum = 0;
//        return;
//    }else{
//        chNum = _ChNum;
//    }


//    isWorking = true;
//    canSendSuspend = true;
//    isSuspend = false;
//    canMBCounter = 0;
//    canHeadIndex = 0;
//    canTailIndex = 0;
//    if(RBCAN_StartThread() == false){
//        isWorking = false;
//        FILE_LOG(logERROR) << "CAN hardware initialize = FAIL";
//    }else{
//        FILE_LOG(logSUCCESS) << "CAN hardware initialize = OK";
//    }

    canTotalIndex = 0;
    canHeadIndex = 0;
    canTailIndex = 0;
}
// --------------------------------------------------------------------------------------------- //
RBCAN::~RBCAN(){
    isWorking = false;
    usleep(100*1000);

//    for(int i=0; i<chNum; i++)
//        CAN_Close(canHandler[i]);
}
// --------------------------------------------------------------------------------------------- //
void RBCAN::Finish(){
    isWorking = false;
    usleep(100*1000);

#ifdef __RB_USE_LAN2CAN__
    LanTCPClientClose();
#endif
//    for(int i=0; i<chNum; i++)
//        CAN_Close(canHandler[i]);
}

// --------------------------------------------------------------------------------------------- //
//void RBCAN::RBResetCAN(){

//    isSuspend = true;
//    usleep(10);
//    for(int i=0; i<chNum; i++)
//        CAN_Close(canHandler[i]);

//    int oknum = 0;
//    for(int i=0; i<MAX_SEARCH_CHANNEL; i++){
//        char filePath[30];
//        sprintf(filePath, "/dev/pcanusb%d", i);
//        canHandler[oknum] = NULL;
//        canHandler[oknum] = LINUX_CAN_Open(filePath, O_RDWR);

//        if(canHandler[oknum] != NULL){
//            if(CAN_Init(canHandler[oknum], CAN_BAUD_1M, CAN_INIT_TYPE_ST) !=  CAN_ERR_OK){
//                FILE_LOG(logWARNING) << "Fail to setting CAN device (" << i << ")";
//            }else{
//                oknum++;
//                if(oknum >= chNum){
//                    break;
//                }
//            }
//        }else{
//            FILE_LOG(logWARNING) << "Fail to open CAN device (" << i << ")";
//        }
//    }

//    isSuspend = false;
//}

// --------------------------------------------------------------------------------------------- //
bool RBCAN::IsWorking(){
    return isWorking;
}
// --------------------------------------------------------------------------------------------- //
int RBCAN::RBCAN_StartThread(void)
{
    if(rt_task_create(&canReadThreadHandler, "RCR_CAN_READ", 0, 95, 0) == 0){
//    if(rt_task_create(&canReadThreadHandler, "RBCAN_READ_TASK", 0, 95, 0) == 0){
        cpu_set_t aCPU;
        CPU_ZERO(&aCPU);
        CPU_SET(1, &aCPU);
        if(rt_task_set_affinity(&canReadThreadHandler, &aCPU) != 0){
            FILE_LOG(logWARNING) << "RBCAN: Read thread set affinity CPU failed..";
        }
        if(rt_task_start(&canReadThreadHandler, &RBCAN_ReadThread, this) == 0){

        }else{
            FILE_LOG(logERROR) << "RBCAN: Read thread Creation Error";
            return false;
        }
    }else{
        FILE_LOG(logERROR) << "RBCAN: Read thread Creation Error";
        return false;
    }

//    int threadID = pthread_create(&canWriteThreadHandler, NULL, &RBCAN_WriteThread, this);
//    if(threadID < 0){
//        FILE_LOG(logERROR) << "RBCAN: Write thread Creation Error";
//        return false;
//    }
//    pthread_setname_np(canWriteThreadHandler, "RCR_CAN_WRITE");
    return true;
}
// --------------------------------------------------------------------------------------------- //
int RBCAN::RBCAN_WriteData(RBCAN_MB mb)
{
//    canTotalMB[canTotalIndex] = _mb;
//    canTotalIndex = (canTotalIndex+1);

//    return true;

    canTotalMB[canHeadIndex] = mb;
    canHeadIndex = (canHeadIndex+1)%RBCAN_MAX_NUM;

    return mb.dlc;
}
// --------------------------------------------------------------------------------------------- //
int RBCAN::RBCAN_WriteDataDirectly(RBCAN_MB _mb)
{
    RBCAN_WriteData(_mb);
//    unsigned char send_byte[21];
//    send_byte[0] = 0x24;            // HEADER
//    send_byte[1] = (14)&0xFF;       // Total data length
//    send_byte[2] = (14>>8)&0xFF;    // Total data length
//    send_byte[3] = 0;               // From (Master : 0, Slave(CAN) : 1, General interface board : 2)
//    send_byte[4] = 1;               // To (Master : 0, Slave(CAN) : 1, General interface board : 2)
//    send_byte[5] = 0;               // Data type (CAN Data : 0, Setting Data : 1)
//    // data
//    send_byte[6] = (1)&0xFF;        // Number of CAN message
//    send_byte[7] = (1>>8)&0xFF;     // Number of CAN message
//    send_byte[8] = _mb.channel;         // Channel
//    send_byte[9] = (_mb.id)&0xFF;       // ID
//    send_byte[10]= (_mb.id>>8)&0xFF;    // ID
//    send_byte[11]= _mb.dlc;             // Data Length Code
//    for(int i=0; i<8; i++){
//        send_byte[12+i] = _mb.data[i];  // Data Fixed size(8bytes)
//    }
//    send_byte[20] = 0x25;           // FOOTER
//    write(LAN_CLIENT_FD, send_byte, 21);
    return true;//findwarning

}
// --------------------------------------------------------------------------------------------- //
int RBCAN::RBCAN_ReadData(pRBCAN_MB _mb)
{
    unsigned int index = RBCAN_GetMailBoxIndex(_mb->id);

    _mb->dlc = canReadMB[index].dlc;
    memcpy(_mb->data, canReadMB[index].data, canReadMB[index].dlc);
    _mb->status = canReadMB[index].status;

    canReadMB[index].status = RBCAN_NODATA;
    return true;
}
// --------------------------------------------------------------------------------------------- //
int RBCAN::RBCAN_AddMailBox(unsigned int _id)
{
    unsigned char i;

    if(canMBCounter >= RBCAN_MAX_NUM){
        FILE_LOG(logWARNING) << "Over the CAN mail box";
        return false;
    }else{
        if(RBCAN_GetMailBoxIndex(_id) == canMBCounter){
            canReadMB[canMBCounter].id = _id;                           // CAN id assign
            canReadMB[canMBCounter].dlc = 0x00;                         // Data Length Code
            for(i=0; i<8; i++) canReadMB[canMBCounter].data[i] = 0x00;	// Data init.
            canReadMB[canMBCounter].status = RBCAN_NODATA;          // Initial MB status = No Data
            canMBCounter++;
            return true;
        }else{
            if(_id != 0x00){
                FILE_LOG(logWARNING) << "CAN ID(" << _id << ") is already assigned";
            }
            return false;
        }
    }
}
// --------------------------------------------------------------------------------------------- //
int RBCAN::RBCAN_GetMailBoxIndex(unsigned int _id)
{
    for(int i=0 ; i<canMBCounter; i++) if(canReadMB[i].id == _id) return i;
    return canMBCounter;
}
// --------------------------------------------------------------------------------------------- //
int RBCAN::RBCAN_WriteEnable(int _suspend)
{
    canSendSuspend = _suspend;
    return canSendSuspend;
}
// --------------------------------------------------------------------------------------------- //


void RBCAN::RBCAN_ClearBuffer(){
    canTotalIndex = 0;
}



void RBCAN::RBCAN_SendData(){

    static int cnt = 250 ;
    static unsigned char send_byte[1024];





    if(lanHandler->ConnectionStatus == true){
        cnt--;

        // Send CAN Data ---------------------
        //if(canTotalIndex != 0){
        if(canHeadIndex != canTailIndex){
            cnt = 250;
            canTotalIndex = canHeadIndex - canTailIndex;
            if(canTotalIndex < 0){
                canTotalIndex += RBCAN_MAX_NUM;
            }
            if(canTotalIndex > 30)
            {
                printf("canTotalIndex > 30 : %d\n", canTotalIndex);
            }


            send_byte[0] = 0x24;            // HEADER
            unsigned int TotalDataLength = 4 + 2 + 12 * canTotalIndex;
            send_byte[1] = (TotalDataLength)&0xFF;       // Total data length
            send_byte[2] = (TotalDataLength>>8)&0xFF;    // Total data length
            send_byte[3] = 0;               // From (Master : 0, Slave(CAN) : 1, General interface board : 2)
            send_byte[4] = 1;               // To (Master : 0, Slave(CAN) : 1, General interface board : 2)
            send_byte[5] = 0;               // Data type (CAN Data : 0)
            send_byte[6] = (canTotalIndex)&0xFF;
            send_byte[7] = (canTotalIndex>>8)&0xFF;

            for(int idx = 0; idx < canTotalIndex; idx++){
                // data
                int temp_idx = canTailIndex;
//                send_byte[8 + (12 * idx)] = canTotalMB[idx].channel;// Channel
//                unsigned int _CANID = canTotalMB[idx].id;

//                send_byte[9 + (12 * idx)] = (_CANID)&0xFF;     // ID
//                send_byte[10 + (12 * idx)]= (_CANID>>8)&0xFF;   // ID
//                send_byte[11 + (12 * idx)]= canTotalMB[idx].dlc;               // Data Length Code
//                for(int i=0; i<8; i++){
//                    send_byte[12+i + (12 * idx)] = canTotalMB[idx].data[i];  // Data Fixed size(8bytes)
//                }
                send_byte[8 + (12 * idx)] = canTotalMB[temp_idx].channel;// Channel
                unsigned int _CANID = canTotalMB[temp_idx].id;

                send_byte[9 + (12 * idx)] = (_CANID)&0xFF;     // ID
                send_byte[10 + (12 * idx)]= (_CANID>>8)&0xFF;   // ID
                send_byte[11 + (12 * idx)]= canTotalMB[temp_idx].dlc;               // Data Length Code
                for(int i=0; i<8; i++){
                    send_byte[12+i + (12 * idx)] = canTotalMB[temp_idx].data[i];  // Data Fixed size(8bytes)
                }
                canTailIndex = (canTailIndex+1)%RBCAN_MAX_NUM;
            }

            send_byte[TotalDataLength+3-1] = 0x25;           // FOOTER

            int now_tx_size = TotalDataLength + 3;

//            FILE_LOG(logSUCCESS) << "send can : " << now_tx_size;
            RBLANComm::RBLANComm_WriteData(send_byte, now_tx_size);

//            auto& s = send_byte;
//            std::printf("%d %d %d %d: ", s[3], s[4], s[5], int(s[6] | s[7] << 8));
//            for (int i = 0; i < canTotalIndex; i++) {
//                int cid = int(s[9 + (12 * i)] | s[10 + (12 * i)] << 8);
//                std::printf("%d %d, ", cid, s[11 + (12 * i)]);
//            }
//            std::printf("\n");
        }else{
            if(cnt <= 0)
            {
                send_byte[0] = 0x24;           // HEADER
                send_byte[1] = 0x01;           // FOOTER
                send_byte[2] = 0x00;           // FOOTER
                send_byte[3] = 0x25;           // FOOTER
                RBLANComm::RBLANComm_WriteData(send_byte, 4);
             //    printf("RBLANComm: dummy %d %d %d %d \n",send_byte[0] ,send_byte[1] ,send_byte[2] ,send_byte[3] );
                cnt =250;
            }
        }
    }else{
        //canTotalIndex = 0;
        canHeadIndex = canTailIndex = 0;
    }
}

int cnt_enc = 0;

void RBCAN::RBCAN_MessageDecode(int id, unsigned char *data, int dlc){
     static char msg[30];
    //cout << "message decode: " << id << endl;
    if(id >= 0x50 && id <= 0x5B){
        // encoder data

        if(sharedData->RaisimSimulation == false) {
            int mc = id-0x50;

            int temp_enc1 = (int)((data[0]) | (data[1]<<8) | (data[2]<<16));
            if(temp_enc1 & 0x800000){
                temp_enc1 |= 0xFF000000;
            }
            int temp_cur = (int)((short)(data[3] | (data[4]<<8)));
            _DEV_MC[mc].EncoderValue = temp_enc1;

            //double tempP = (double)temp_enc1/_DEV_MC[mc].PPR + _DEV_MC[mc].homeJointOffset;
            double tempP = (double)temp_enc1/_DEV_MC[mc].PPR + _DEV_MC[mc].homeJointOffset + _DEV_MC[mc].homeJointOffset_mult;

            double vel_pre = _DEV_MC[mc].CurrentVelocity;

            _DEV_MC[mc].CurrentVelocity = (tempP-_DEV_MC[mc].CurrentPosition)/(0.001*RT_TIMER_PERIOD_MS);
            _DEV_MC[mc].CurrentAcc = (_DEV_MC[mc].CurrentVelocity-vel_pre);

            //cout << "vel : " << _DEV_MC[mc].CurrentVelocity << endl;

            _DEV_MC[mc].CurrentPosition = tempP;
            _DEV_MC[mc].MeasuredCurrent = temp_cur*_DEV_MC[mc].TPC;

            _DEV_MC[mc].CurrentStatus.b.RUN = (data[5] >> 0) & 0x01;
            _DEV_MC[mc].CurrentStatus.b.MOD = (data[5] >> 1) & 0x01;
            _DEV_MC[mc].CurrentStatus.b.JAM = (data[5] >> 2) & 0x01;
            _DEV_MC[mc].CurrentStatus.b.CUR = (data[5] >> 3) & 0x01;
            _DEV_MC[mc].CurrentStatus.b.BIG = (data[5] >> 4) & 0x01;
            _DEV_MC[mc].CurrentStatus.b.INP = (data[5] >> 5) & 0x01;
            _DEV_MC[mc].CurrentStatus.b.PS1 = (data[5] >> 6) & 0x01;
            _DEV_MC[mc].CurrentStatus.b.PS2 = (data[5] >> 6) & 0x01;
            _DEV_MC[mc].CurrentStatus.b.MT_ERR = (data[5] >> 7) & 0x01;

            if(((data[6]>>7)&0x01) == 0){
                _DEV_MC[mc].BoardTemperature = data[6];
            }else{
                _DEV_MC[mc].MotorTemperature = (int)(data[6]&0x7F);
            }
        }
//        if(mc == 2) {
//            FILE_LOG(logDEBUG) << "cnt : " << cnt_enc << " initialized";
//            cnt_enc++;
//        }
    }

    else if(id >= 0x60 && id <= 0x6B){
        // encoder data

        int mc = id-0x60;

        int temp_cur = (int)((short)(data[0] | (data[1]<<8)));
        _DEV_MC[mc].CurrentReference_A_low = (double)temp_cur*0.001*_DEV_MC[mc].TPC;

    }
    else if(id >= 0x30 && id <= 0x3B){
        // board status

        int mc = id-0x30;

        _DEV_MC[mc].CurrentStatus.B[0] = data[0];
        _DEV_MC[mc].CurrentStatus.B[1] = data[1];
        _DEV_MC[mc].CurrentStatus.B[2] = data[2];
        _DEV_MC[mc].BoardTemperature = data[3];
        _DEV_MC[mc].MotorTemperature = data[4];

    }else if(id >= 0x20 && id <= 0x2B){
        int mc = id-0x20;

//        FILE_LOG(logERROR) << mc;
        if(data[0] == 0x01){
            // can check return
            _DEV_MC[mc].ConnectionStatus = true;
//            _DEV_MC[mc].CurrentStatus.b.CAN = _DEV_MC[mc].ConnectionStatus;
            FILE_LOG(logSUCCESS) << "RBMC:: bno " << id-0x20 << " initialized";
        }

        if(data[0] == 0xD2) {
            int foc_kp = (int)((short)(data[1] | (data[2]<<8)));
            int foc_ki = (int)((short)(data[3] | (data[4]<<8)));

            FILE_LOG(logSUCCESS) << "RBMC:: bno " << id-0x20 << ", FOC KP : " << foc_kp << ", FOC KI : " << foc_ki << " set!!!";
        }

        if(data[0] == 0xD5) {
            int pos_kp = (int)((short)(data[1] | (data[2]<<8)));
            int pos_ki = (int)((short)(data[3] | (data[4]<<8)));
            int pos_kd = (int)((short)(data[5] | (data[6]<<8)));

            FILE_LOG(logSUCCESS) << "RBMC:: bno " << id-0x20 << ", POS KP : " << pos_kp << ", POS KI : " << pos_ki << ", POS KD : " << pos_kd << " set!!!";
        }

    }
    else if(id >= 900 && id <= 911){

        sprintf(msg, "CPU %d RST", id-900);
//        RCR_LOG(LOG_TYPE_FATAL, msg);
    }else if(id >= 950 && id <= 961){
        //        char msg[30];
        //        double del_gap = ((double)((int)(data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24))))/60000.;
        //        sprintf(msg, "CAN %d CNT ERR : %.1fms", id-950, del_gap);
        //        RCR_LOG(LOG_TYPE_ERROR, msg);


        if(data[0] == 0){
            double del_gap = ((double)((int)(data[1] | (data[2] << 8) | (data[3] << 16) | (data[4] << 24))))/100.;
            FILE_LOG(logWARNING) <<"CAN "<<id-950<<" CNTE "<<del_gap<<" ms";
            CAN_COUNT_ERR_index = 0;
//            sprintf(msg, "CAN %d CNT ERR : %.1fms", id-950, del_gap);
//            RCR_LOG(LOG_TYPE_ERROR, msg);
//            can_timeout_err[id-950]++;
        }else if(data[0] == 1){
            int index = data[1];
            int num = (int)(data[2] | (data[3]<<8) | (data[4]<<16) | (data[5]<<24));
            FILE *fp = fopen("hw_jitter.dat", "a");
            fprintf(fp, "[%d] jitter : %d us   %d\n", id-950, (index+1)*10, num);
            fclose(fp);
            //  cout << "[" << id-950 << "] jitter : " << (index+1)*10 << " us   " << num << endl;
        }

    }




//    if(id == 0x80){
//        double q0 = (double)((short)((data[1] << 8) | data[0])) / 30000.0;
//        double q1 = (double)((short)((data[3] << 8) | data[2])) / 30000.0;
//        double q2 = (double)((short)((data[5] << 8) | data[4])) / 30000.0;
//        double q3 = (double)((short)((data[7] << 8) | data[6])) / 30000.0;

//        double phi = atan2(2 * (q2*q3 + q0*q1), 1 - 2 * (q1*q1 + q2*q2))*57.2957914;
//        double theta = -1 * asin(2 * (q1*q3 - q0*q2))*57.2957914;
//        double psi = atan2(2 * (q1*q2 + q0*q3), 1 - 2 * (q2*q2 + q3*q3))*57.2957914;

//        sharedData->IMU[0].Q[0] = q0;
//        sharedData->IMU[0].Q[1] = q1;
//        sharedData->IMU[0].Q[2] = q2;
//        sharedData->IMU[0].Q[3] = q3;

//        sharedData->IMU[0].Yaw = psi;
//        sharedData->IMU[0].Pitch = theta;
//        sharedData->IMU[0].Roll = phi;
//        sharedData->IMU[0].NO_RESPONSE_CNT = 0;

//        Imu_yaw = psi;
//    }

#ifdef CheapIMU
    if(id == 0x81){
        double roll_vel = (double)((short)((data[1] << 8) | data[0])) / 65.0;
        double pitch_vel = (double)((short)((data[3] << 8) | data[2])) / 65.0;
        double yaw_vel = (double)((short)((data[5] << 8) | data[4])) / 65.0;

        sharedData->IMU[0].RollVel = roll_vel;
        sharedData->IMU[0].PitchVel = pitch_vel;
        sharedData->IMU[0].YawVel = yaw_vel;

//        FILE_LOG(logSUCCESS) << "gyro";


    }else if(id == 0x83){
        double acc_x = (double)((short)((data[1] << 8) | data[0]))/100.0;
        double acc_y = (double)((short)((data[3] << 8) | data[2]))/100.0;
        double acc_z = (double)((short)((data[5] << 8) | data[4]))/100.0;

        sharedData->IMU[0].AccX = acc_x;
        sharedData->IMU[0].AccY = acc_y;
        sharedData->IMU[0].AccZ = acc_z;

//        FILE_LOG(logSUCCESS) << "Acc";
    }
#endif

}


// LAN2CAN ====================================================


int RBCAN::LanTCPReady(int _port){
    LAN_SERVER_FD = socket(AF_INET, SOCK_STREAM, 0);
    if(LAN_SERVER_FD == -1)
        return false;

    memset(&ServerAddr, 0, sizeof(ServerAddr));
    ServerAddr.sin_family     = AF_INET;
    ServerAddr.sin_port       = htons(_port);
    ServerAddr.sin_addr.s_addr= htonl(INADDR_ANY);

    int optval = 1;
//    setsockopt(LAN_SERVER_FD, IPPROTO_TCP, TCP_NODELAY, &optval, sizeof(optval));
    setsockopt(LAN_SERVER_FD, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval));
    setsockopt(LAN_SERVER_FD, SOL_SOCKET, SO_REUSEPORT, &optval, sizeof(optval));

    struct linger optlinger;
    optlinger.l_onoff = 1;
    optlinger.l_linger = 3600;	// 3600sec
    setsockopt(LAN_SERVER_FD, SOL_SOCKET, SO_LINGER, (char*)&optlinger, sizeof(optlinger));

    if(bind(LAN_SERVER_FD, (struct sockaddr*)&ServerAddr, sizeof(ServerAddr)) == -1)
        return false;
    return true;
}


int RBCAN::LanTCPAccept(void){
    unsigned int client_addr_size;

    FILE_LOG(logINFO) << "Server waits for client connection..!!";

    if(listen(LAN_SERVER_FD, 5) == -1)
        return false;

    client_addr_size = sizeof(ClientAddr);
    LAN_CLIENT_FD = accept(LAN_SERVER_FD, (struct sockaddr*)&ClientAddr, &client_addr_size);

    if(LAN_CLIENT_FD == -1)
        return false;

    return true;
}


int RBCAN::CreateSocket(const char *addr, int port){
    LAN_CLIENT_FD = socket(AF_INET, SOCK_STREAM, 0);
    if(LAN_CLIENT_FD == -1){
        return false;
    }
    ClientAddr.sin_addr.s_addr = inet_addr(addr);
    ClientAddr.sin_family = AF_INET;
    ClientAddr.sin_port = htons(port);

    int optval = 1;
//    setsockopt(LAN_CLIENT_FD, IPPROTO_TCP, TCP_NODELAY, &optval, sizeof(optval));
    setsockopt(LAN_CLIENT_FD, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval));
    setsockopt(LAN_CLIENT_FD, SOL_SOCKET, SO_REUSEPORT, &optval, sizeof(optval));

    return true;
}

int RBCAN::Connect2Server(){
    if(connect(LAN_CLIENT_FD, (struct sockaddr*)&ClientAddr, sizeof(ClientAddr)) < 0){
        return false;
    }
    FILE_LOG(logSUCCESS) << "Client connect to server..!!";

    return true;
}


int RBCAN::LanTCPClientClose(){
    return close(LAN_CLIENT_FD);
}
int RBCAN::LanTCPServerClose(){
    return close(LAN_SERVER_FD);
}

