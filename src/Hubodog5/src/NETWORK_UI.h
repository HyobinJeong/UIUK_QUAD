#ifndef NETWORK_UI_H
#define NETWORK_UI_H

#include "RBLog.h"

#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>

#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>


#define UI_LAN_DATA_SIZE    40000


class NETWORK_UI
{
public:
    NETWORK_UI();

//    int     NETWORK_UI_COMMAND;

    int     ConnectionStatus;
    int     RXDataReadIndex;
    int     RXDataStoreIndex;
    char    RXData[UI_LAN_DATA_SIZE];

    int     LAN_SERVER_FD;
    int     LAN_CLIENT_FD;

    struct sockaddr_in  ServerAddr;
    struct sockaddr_in  ClientAddr;

    int LanTCPReady(int _port = 1977);
    int LanTCPAccept();
    int CreateSocket(const char *addr, int port);
    int Connect2Server();

    int LanTCPClientClose();
    int LanTCPServerClose();


private:
    int         isWorking;
    ulong       UI_ReadThreadHandler;

    int         UI_StartThread();
    static void *UI_ReadThread(void *arg);





};

#endif // NETWORK_UI_H
