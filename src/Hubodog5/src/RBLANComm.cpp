//#include "RBCommon.h"
#include <iostream>
#include <string.h>

#include "RBLANComm.h"
//#include "CommonHeader.h"

#include "RBCAN.h"
#include <vector>

#define RBLANCOMM_PACKET_DEBUG 0

extern pRBCORE_SHM sharedData;
extern RBCAN        *canHandler;
extern int IS_NEW_CAN_DATA;

// define static variables
int RBLANComm::LAN_CLIENT_FD = 0;

using namespace std;

RBLANComm::RBLANComm(){
    ConnectionStatus = false;
    //pthread_create_with_affinity(RBLANComm_ConnectionThread,0,"RCR_RBLANCOMM",ConnectionThreadHandler,this);
    threadID_connectionThread = pthread_create(&ConnectionThreadHandler, NULL, &RBLANComm_ConnectionThread, this);
}

RBLANComm::~RBLANComm(){
    ConnectionStatus = false;

    printf("[RBLANComm] Shutdown Network\n");
    shutdown(LAN_CLIENT_FD,2);
    printf("[RBLANComm] Closing Network\n");
    //close(LAN_CLIENT_FD);
}

extern int init_state;

// This thread only uses for connection management
void *RBLANComm::RBLANComm_ConnectionThread(void *arg){
    static int cnt = 0;
    RBLANComm *lan = (RBLANComm *)arg;
    cnt++;
    printf("RBLANComm_ConnectionThread cnt = %d\n",cnt);
    while(_IS_WORKING){        
        if(lan->ConnectionStatus == false){
            // if client is not connected
            if(lan->LAN_CLIENT_FD == 0){
                if(lan->CreateSocket("10.0.1.1", 1977) == true){
                    FILE_LOG(logSUCCESS) << "RBLANComm:: succeed to create socket";
                }
            }else{
                if(lan->Connect2Server() == true){
                    lan->ConnectionStatus = true;
                    canHandler->RBCAN_ClearBuffer();
                    FILE_LOG(logSUCCESS) << "RBLANComm:: succeed to connect server";
                    init_state = 1;
                }
            }
        }
        usleep(1000*1000);
    }
    return NULL;
}


void RBLANComm::RBLANComm_WriteData(const void *buf, size_t len){
   write(LAN_CLIENT_FD, buf, len);
}


void RBLANComm::RBLANComm_ReadData(){
    static std::vector<unsigned char>  totalLANData;

    static unsigned char rzdata[RX_DATA_SIZE];
    static  unsigned char tempPacketData[RX_DATA_SIZE];
    int tcp_size = 0;

    if(ConnectionStatus == true)
    {

        tcp_size = recv(LAN_CLIENT_FD, rzdata, RX_DATA_SIZE, 0);
        totalLANData.insert(totalLANData.end(),&rzdata[0],&rzdata[tcp_size]);
        tcp_size = totalLANData.size();

        if(tcp_size > 0)
        {

            while(_IS_WORKING)
            {
                tcp_size = totalLANData.size();
                if(tcp_size < 4){
                    return;
                }

                int packet_length = 0;
                if(totalLANData[0] == 0x24){
                    packet_length = (unsigned short)(totalLANData[1] | (totalLANData[2]<<8));
                    if(packet_length < 0){
                        FILE_LOG(logERROR) << "RBLANComm:: packet length under zero";
                        //ERROR
                    }else if(packet_length > 40000){
                        FILE_LOG(logERROR) << "RBLANComm:: packet length too big";
                    }


                    if(tcp_size < packet_length +3){
                        return;
                    }

                    if(packet_length == 1){
                        if(totalLANData[3] == 0x25){
                            //printf(">> Reply To PC\n");
                            totalLANData.erase(totalLANData.begin(), totalLANData.begin()+4);
                            continue;
                        }
                    }

                    if(tcp_size >= packet_length +3)
                    {
                        memcpy(tempPacketData, &totalLANData[0], packet_length+3);
                        totalLANData.erase(totalLANData.begin(),totalLANData.begin()+( packet_length+3));
                        if((tempPacketData[0] == 0x24) && (tempPacketData[packet_length + 3 - 1] == 0x25)){
                            int data_type = tempPacketData[5];
                            switch(data_type){

                            case 0x00:
                            {
                                // CAN Data
                                int num = (short)(tempPacketData[6] | (tempPacketData[7]<<8));
                                //                                cout << "message num: " << num << endl;
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
//FILE_LOG(logWARNING) << "ch: " << ch_type;
                                    canHandler->RBCAN_MessageDecode(id, data, dlc);
                                    //                                    cout << endl;
                                }
//#ifdef RBLANCOMM_PACKET_DEBUG
//                                printf("[Packet,0x00,length=%d]\n",packet_length+3);
//                                for(int i = 0 ; i < packet_length+3 ; i++)
//                                {
//                                    printf("0x%x ",tempPacketData[i]);
//                                }
//                                printf("\n");
//#endif
                                break;
                            }
                            case 0x01:
                            {
                                float Vin = (float)((tempPacketData[6] | (tempPacketData[7]<<8))/100.0);
                                float Iin = (float)(tempPacketData[8])/5.0;

                                sharedData->Vin = Vin;
                                sharedData->Iin = Iin;

                                //cout << "Vin : " << Vin << ", Iin : " << Iin << endl;
                                break;
                            }
                            default:
//#ifdef RBLANCOMM_PACKET_DEBUG
//                                printf("[Packet,default,length=%d]\n",packet_length+3);
//                                for(int i = 0 ; i < packet_length+3 ; i++)
//                                {
//                                    printf("0x%x ",tempPacketData[i]);
//                                }
//                                printf("\n");
//#endif

                                break;
                            }

                        }else{
                            FILE_LOG(logERROR) << "RBLANComm:: header footer not match";
//                            if(tempPacketData[0] != 0x24){
//                                FILE_LOG(logERROR) << "Header Not Match";
//                            }
//                            if(tempPacketData[packet_length+3-1] != 0x25){
//                                FILE_LOG(logERROR) << "Footer Not Match";
//                            }
                            FILE_LOG(logERROR) << "SIZE : " << packet_length << ", " << tcp_size;
                            for(int kkk = 0; kkk<packet_length+3-1; kkk++){
                                printf("0x%x ",tempPacketData[kkk]);
                                //cout << tempPacketData[kkk] << " " ;
                            }
                            printf("\n");printf("\n");printf("\n");
                            //#if RBLANCOMM_PACKET_DEBUG
//                            for(std::vector<unsigned char>::iterator i  = totalLANData.begin() ; i < totalLANData.end(); i++ )
//                            {
//                                if(*i == '$')
//                                {
//                                    break;
//                                }
//                                else
//                                {
//                                    printf("0x%x ",0xff&*i);
//                                    totalLANData.erase(i);
//                                }
//                            }
                            totalLANData.clear();

                            //#endif

                        }
                    }

                }
                else {
                    //ERROR
                    totalLANData.erase(totalLANData.begin(), totalLANData.begin()+1);
                }
            }
        }
        else{
            //cout << "no data" << endl;
        }
    }
    else{
        usleep(10*1000);
    }
}




// Internal Functions -------------------------------------------------------------------------------------------------



int RBLANComm::CreateSocket(const char *addr, int port){
    LAN_CLIENT_FD = socket(AF_INET, SOCK_STREAM, 0);
    if(LAN_CLIENT_FD == -1){
        return false;
    }
    ClientAddr.sin_addr.s_addr = inet_addr(addr);
    ClientAddr.sin_family = AF_INET;
    ClientAddr.sin_port = htons(port);

    int optval = 1;
    if(setsockopt(LAN_CLIENT_FD, IPPROTO_TCP, TCP_NODELAY, &optval, sizeof(optval)) == -1){
        return false;
    }
    if(setsockopt(LAN_CLIENT_FD, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval)) == -1){
        return false;
    }
    if(setsockopt(LAN_CLIENT_FD, SOL_SOCKET, SO_REUSEPORT, &optval, sizeof(optval)) == -1){
        return false;
    }


    return true;
}
int RBLANComm::Connect2Server(){
    if(connect(LAN_CLIENT_FD, (struct sockaddr*)&ClientAddr, sizeof(ClientAddr)) == -1){
        return false;
    }
    // non blocking mode
    //  int flags = fcntl(LAN_CLIENT_FD, F_GETFL, 0);
    //  fcntl(LAN_CLIENT_FD, F_SETFL, flags | O_NONBLOCK);
    return true;
}
int RBLANComm::LanTCPClientClose()
{
    return close(LAN_CLIENT_FD);
}
