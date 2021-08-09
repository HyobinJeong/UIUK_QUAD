#include "RBCAN.h"
#include "motion/RBMotion.h"

extern FILE *save_fp;
extern int save_start;
extern unsigned int hello_reply;
unsigned int hello_reply_old;
TCP_BOARD_DATA_STRUC RCR_EE_DATA;

using namespace std;
// --------------------------------------------------------------------------------------------- //
void RBCAN::RBCAN_ReadThread(void *_arg){
    RBCAN *rbCAN = (RBCAN *)_arg;

#ifdef __RB_USE_LAN2CAN__
    unsigned int tcp_status = 0x00;
    int tcp_size = 0;
    char totalLANData[RX_DATA_SIZE];
    char tempPacketData[RX_DATA_SIZE];
    int connectCnt = 0;
#else
    TPCANRdMsg m;
#endif

    int index;

    while(rbCAN->isWorking == true){
        if(rbCAN->isSuspend == true){
            usleep(10);
            continue;
        }
        for(int i=0; i<rbCAN->chNum; i++){
#ifdef __RB_USE_LAN2CAN__
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
                    FILE_LOG(logERROR) << "Connect to Server Failed..!!";
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
                                                int id = (short)((tempCANData[1]) | tempCANData[2]<<8);
                                                int dlc = tempCANData[3];
                                                unsigned char data[8];
                                                memcpy(data, &tempCANData[4], dlc);


                                                //
                                                if(id == 0x110){
                                                    RCR_EE_DATA.gx = ((double)((short)(tempCANData[4] | (tempCANData[5] << 8))))/65.;
                                                    RCR_EE_DATA.gy = ((double)((short)(tempCANData[6] | (tempCANData[7] << 8))))/65.;
                                                    RCR_EE_DATA.gz = ((double)((short)(tempCANData[8] | (tempCANData[9] << 8))))/65.;
                                                }else if(id == 0x111){
                                                    RCR_EE_DATA.ax = ((double)((short)(tempCANData[4] | (tempCANData[5] << 8))))/1000.;
                                                    RCR_EE_DATA.ay = ((double)((short)(tempCANData[6] | (tempCANData[7] << 8))))/1000.;
                                                    RCR_EE_DATA.az = ((double)((short)(tempCANData[8] | (tempCANData[9] << 8))))/1000.;
                                                    RCR_EE_DATA.is_new_data = 1;
                                                }else if(id == 0x112){
                                                    RCR_EE_DATA.output_voltage = (tempCANData[4] & 0b00111111);
                                                    RCR_EE_DATA.digi_input_A = ((tempCANData[4] >> 7) & 0x01);
                                                    RCR_EE_DATA.digi_input_B = ((tempCANData[4] >> 6) & 0x01);
                                                    RCR_EE_DATA.anal_input_A = ((double)tempCANData[5])/20.;
                                                    RCR_EE_DATA.anal_input_B = ((double)tempCANData[6])/20.;
                                                    RCR_EE_DATA.switch_A = ((tempCANData[7] >> 7) & 0x01);
                                                    RCR_EE_DATA.temp = (tempCANData[7] & 0b01111111);
                                                }else if(id >= 900 && id <= 906){
                                                    char msg[30];
                                                    sprintf(msg, "CPU %d RST", id-900);
                                                    RCR_LOG(LOG_TYPE_FATAL, msg);
                                                }else if(id >= 950 && id <= 956){

                                                    if(tempCANData[4] == 0){
                                                        char msg[30];
                                                        double del_gap = ((double)((int)(tempCANData[5] | (tempCANData[6] << 8) | (tempCANData[7] << 16) | (tempCANData[8] << 24))))/100.0;
//                                                        cout<<"CNT ERR"<<endl;
                                                        sprintf(msg, "CAN %d CNT ERR : %.1fms", id-950, del_gap);
                                                        RCR_LOG(LOG_TYPE_ERROR, msg);
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
                                        case 0x01:
                                        {
                                            break;
                                        }
                                        case 0x02:
                                            break;
                                        case 0x03:
                                            break;
                                        case 0x80:
                                            break;
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

#else
            while(LINUX_CAN_Read_Timeout(rbCAN->canHandler[i], &m, 0) == 0){

                if(m.Msg.ID == 701){
                    int test1, test2, test3;
                    test1 = (int)((m.Msg.DATA[0] | (m.Msg.DATA[1]<<8) | (m.Msg.DATA[2]<<16) | (m.Msg.DATA[3]<<24)));
                    test2 = (int)(m.Msg.DATA[4] | (m.Msg.DATA[5]<<8));// | (m.Msg.DATA[6]<<16) | (m.Msg.DATA[7]<<24)));
                    test3 = (short)(m.Msg.DATA[6] | (m.Msg.DATA[7]<<8));
//                    FILE_LOG(logINFO) << test1 << ", " << test2;

//                    if(save_start){
//                        fprintf(save_fp, "%d, %d, %d\n", test1, test2, test3);
//                        fflush(save_fp);
//                    }
                    continue;
                }

                index = rbCAN->RBCAN_GetMailBoxIndex(m.Msg.ID);

                if(index < rbCAN->canMBCounter){
                    rbCAN->canReadMB[index].id = m.Msg.ID;
                    rbCAN->canReadMB[index].dlc = m.Msg.LEN;
                    memcpy(rbCAN->canReadMB[index].data, m.Msg.DATA, m.Msg.LEN);
                    rbCAN->canReadMB[index].channel = i;
                    if(rbCAN->canReadMB[index].status == RBCAN_NEWDATA)
                        rbCAN->canReadMB[index].status = RBCAN_OVERWRITE;
                    else rbCAN->canReadMB[index].status = RBCAN_NEWDATA;
                }
            }
#endif
        }
        usleep(10);
    }
}
// --------------------------------------------------------------------------------------------- //
void *RBCAN::RBCAN_WriteThread(void *_arg){
    RBCAN *rbCAN = (RBCAN *)_arg;
#ifndef __RB_USE_LAN2CAN__
    TPCANMsg m;
#endif
//    int index = 0;//findwarning

    while(rbCAN->isWorking){
        if(rbCAN->isSuspend == true){
            usleep(3);
            continue;
        }
        if(rbCAN->canSendSuspend == true){
            if(rbCAN->canHeadIndex != rbCAN->canTailIndex){
//                index = rbCAN->canTailIndex;

#ifndef __RB_USE_LAN2CAN__
                m.ID = rbCAN->canWriteMB[index].id;
                m.LEN = rbCAN->canWriteMB[index].dlc;
                memcpy(m.DATA, rbCAN->canWriteMB[index].data, rbCAN->canWriteMB[index].dlc);

                LINUX_CAN_Write_Timeout(rbCAN->canHandler[rbCAN->canWriteMB[index].channel], &m, 0);
#endif
                rbCAN->canTailIndex = (rbCAN->canTailIndex + 1) % RBCAN_MAX_MB;
            }
        }
        usleep(5);
    }
    return NULL;
}
// --------------------------------------------------------------------------------------------- //


// --------------------------------------------------------------------------------------------- //
RBCAN::RBCAN(int _ChNum){
    if(_ChNum > RBCAN_MAX_CAN_CHANNEL){
        FILE_LOG(logERROR) << "Over the maximum CAN channel [" << RBCAN_MAX_CAN_CHANNEL << " ]";
        chNum = 0;
        return;
    }else{
        chNum = _ChNum;
    }



#ifdef __RB_USE_LAN2CAN__
    isWorking = true;
    canSendSuspend = true;
    isSuspend = false;
    canMBCounter = 0;
    canHeadIndex = 0;
    canTailIndex = 0;
    if(RBCAN_StartThread() == false){
        isWorking = false;
        FILE_LOG(logERROR) << "CAN hardware initialize = FAIL";
    }else{
        FILE_LOG(logSUCCESS) << "CAN hardware initialize = OK";
    }
#else
    int oknum = 0;
    int okflag = false;
    for(int i=0; i<MAX_SEARCH_CHANNEL; i++){
        char filePath[30];
        sprintf(filePath, "/dev/pcanusb%d", i);
        canHandler[oknum] = NULL;
        canHandler[oknum] = LINUX_CAN_Open(filePath, O_RDWR);

        if(canHandler[oknum] != NULL){
            if(CAN_Init(canHandler[oknum], CAN_BAUD_1M, CAN_INIT_TYPE_ST) !=  CAN_ERR_OK){
                FILE_LOG(logWARNING) << "Fail to setting CAN device (" << i << ")";
            }else{
                oknum++;
                if(oknum >= chNum){
                    okflag = true;
                    break;
                }
            }
        }else{
            FILE_LOG(logWARNING) << "Fail to open CAN device (" << i << ")";
        }
    }


    if(okflag == true){
        isWorking = true;
        canSendSuspend = true;
        isSuspend = false;
        canMBCounter = 0;
        canHeadIndex = 0;
        canTailIndex = 0;

        if(RBCAN_StartThread() == false){
            isWorking = false;
            FILE_LOG(logERROR) << "CAN hardware initialize = FAIL";
        }else{
            FILE_LOG(logSUCCESS) << "CAN hardware initialize = OK";
        }
    }else{
        isWorking = false;
        FILE_LOG(logERROR) << "CAN hardware initialize = FAIL";
    }
#endif
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
int RBCAN::RBCAN_WriteData(RBCAN_MB _mb)
{
#ifdef __RB_USE_LAN2CAN__
    canTotalMB[canTotalIndex] = _mb;
    canTotalIndex = (canTotalIndex+1);
#else
    canWriteMB[canHeadIndex] = _mb;
    canHeadIndex = (canHeadIndex+1) % RBCAN_MAX_MB;
#endif

    return true;
}
// --------------------------------------------------------------------------------------------- //
int RBCAN::RBCAN_WriteDataDirectly(RBCAN_MB _mb)
{
#ifdef __RB_USE_LAN2CAN__
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
#else
    TPCANMsg m;
    m.ID = _mb.id;
    m.LEN = _mb.dlc;
    m.MSGTYPE = MSGTYPE_STANDARD;
    memcpy(m.DATA, _mb.data, m.LEN);

    if(LINUX_CAN_Write_Timeout(canHandler[_mb.channel], &m, 0)){
        return false;
    }
    return true;
#endif
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

    if(canMBCounter >= RBCAN_MAX_MB){
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
