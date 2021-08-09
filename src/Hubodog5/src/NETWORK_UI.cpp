#include "NETWORK_UI.h"
#include "stdlib.h"

int NETWORK_UI_COMMAND;
int NETWORK_UI_VELOCITY;
int NETWORK_UI_VELOCITY_FLAG;
NETWORK_UI::NETWORK_UI()
{
//    NETWORK_UI_COMMAND = 0;
    if(UI_StartThread() == false){
        isWorking = false;
    }else{
//        FILE_LOG(logSUCCESS) << "NETWORK_UI: Initialized!!";
    }
}

int NETWORK_UI::UI_StartThread(){
    if(LanTCPReady(2000) == true){
        isWorking = true;
        int threadID = pthread_create(&UI_ReadThreadHandler, NULL, &UI_ReadThread, this);
        if(threadID < 0){
//            FILE_LOG(logERROR) << "NETWORK_UI: ReadThread Creation Error!!";
            return false;
        }
    }else{
//        FILE_LOG(logERROR) << "NETWORK_UI: TCP Ready Error!!";
        return false;
    }

    system("adb reverse tcp:2000 tcp:2000");

    return true;
}

int prev_input = 100;
void *NETWORK_UI::UI_ReadThread(void *arg){
    NETWORK_UI *net = (NETWORK_UI *)arg;

    unsigned int tcp_status = 0x00;
    int tcp_size = 0;
    char tempData[UI_LAN_DATA_SIZE];

    while(net->isWorking){
        if(tcp_status == 0x00){     // if client was not connected
            if(net->LanTCPAccept() == true){
                tcp_status = 0x01;  // LanTCPAccept() will wait until client is connected
                net->RXDataReadIndex = net->RXDataStoreIndex = 0;
                net->ConnectionStatus = true;
//                FILE_LOG(logSUCCESS) << "NETWORK_UI: Client is connected..!!";
            }else{
//                FILE_LOG(logERROR) << "NETWORK_UI: Can't accept client connection..!!";
            }
        }else if(tcp_status == 0x01){     // if client was connected
            // tcp_size is bytes of received data.
            // If 0 was returned, it means the connect is closed
            // tcp_size = read(rbLAN->LAN_CLIENT_FD, &rbLAN->RXData[Owner->RXDataStoreIndex], RX_DATA_SIZE);
            tcp_size = read(net->LAN_CLIENT_FD, tempData, UI_LAN_DATA_SIZE);

            if(tcp_size == 0){
                net->LanTCPClientClose();
                net->LAN_CLIENT_FD = NULL;
                tcp_status = 0x00;
//                rbLAN->RXDataReadIndex = rbLAN->RXDataStoreIndex = 0;
                net->ConnectionStatus = false;
//                FILE_LOG(logERROR) << "NETWORK_UI: Client is disconnected..!!";
            }else if(tcp_size > 0){

                if(tempData[0] == 0x0A){
//                    std::cout<<"Tablet RCV CMD : "<<(int)(tempData[1])<<std::endl;
                    NETWORK_UI_COMMAND = (int)(tempData[1]);

                    int variable_intput = (int)(tempData[2]);
//                    std::cout<<"Tablet RCV VEL : "<<variable_intput<<std::endl;
                    if(variable_intput != prev_input){
                        NETWORK_UI_VELOCITY = variable_intput;
                        NETWORK_UI_VELOCITY_FLAG = true;
                    }
                    prev_input = variable_intput;
                }

//                for(int i=0; i<tcp_size; i++){
//                    std::cout << (int)(tempData[i]) << ", ";
//                }
//                std::cout << std::endl;

            }
        }
    }
}



int NETWORK_UI::LanTCPReady(int _port){
    LAN_SERVER_FD = socket(AF_INET, SOCK_STREAM, 0);
    if(LAN_SERVER_FD == -1)
        return false;

    memset(&ServerAddr, 0, sizeof(ServerAddr));
    ServerAddr.sin_family     = AF_INET;
    ServerAddr.sin_port       = htons(_port);
    ServerAddr.sin_addr.s_addr= htonl(INADDR_ANY);

    int optval = 1;
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


int NETWORK_UI::LanTCPAccept(void){
    unsigned int client_addr_size;

//    FILE_LOG(logINFO) << "Server waits for client connection..!!";

    if(listen(LAN_SERVER_FD, 5) == -1)
        return false;

    client_addr_size = sizeof(ClientAddr);
    LAN_CLIENT_FD = accept(LAN_SERVER_FD, (struct sockaddr*)&ClientAddr, &client_addr_size);

    if(LAN_CLIENT_FD == -1)
        return false;

    return true;
}


int NETWORK_UI::CreateSocket(const char *addr, int port){
    LAN_CLIENT_FD = socket(AF_INET, SOCK_STREAM, 0);
    if(LAN_CLIENT_FD == -1){
        return false;
    }
    ClientAddr.sin_addr.s_addr = inet_addr(addr);
    ClientAddr.sin_family = AF_INET;
    ClientAddr.sin_port = htons(port);

    int optval = 1;
    setsockopt(LAN_CLIENT_FD, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval));
    setsockopt(LAN_CLIENT_FD, SOL_SOCKET, SO_REUSEPORT, &optval, sizeof(optval));

    return true;
}

int NETWORK_UI::Connect2Server(){
    if(connect(LAN_CLIENT_FD, (struct sockaddr*)&ClientAddr, sizeof(ClientAddr)) < 0){
        return false;
    }
    FILE_LOG(logSUCCESS) << "Client connect to server..!!";

    return true;
}


int NETWORK_UI::LanTCPClientClose(){
    return close(LAN_CLIENT_FD);
}
int NETWORK_UI::LanTCPServerClose(){
    return close(LAN_SERVER_FD);
}
