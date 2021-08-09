#ifndef RBIMUSENSOR_H
#define RBIMUSENSOR_H

#include <fcntl.h>
#include <termios.h>
#include <math.h>
#include <pthread.h>

#include "RBDataType.h"
#include "RBLog.h"

#include <alchemy/task.h>

#include <QElapsedTimer>
#include <QtDebug>
#include <QTimer>

#define RS232_RECEIVE_DATA_BUFFER_SIZE	2000

enum _IMU_PORT_OPEN_STAT_{
    IMU_PORT_SUCCESS = 0,
    IMU_PORT_OPEN_ERROR,
    IMU_BAUD_SET_ERROR,
    IMU_OTHER_SET_ERROR,
    IMU_THREAD_CREATE_ERROR,
    IMU_CHECK_OVER_ERROR
};


class RBIMUSensor
{
public:
    RBIMUSensor();
    ~RBIMUSensor();

    // RS-232 ----
    uint    StoredDataIndex;
    uint    ReadDataIndex;
    char    NewDataAvailable;
    char    ReceivedData[RS232_RECEIVE_DATA_BUFFER_SIZE];
    char    ReceivedByte;
    char    WantedByte;
    int     RS232DeviceHandler;

    // FOG -----
//    int     FOGNullFlag;
//    float   FOGRoll;
//    float   FOGPitch;
//    float   FOGYaw;

//    float   FOGRollVel;
//    float   FOGPitchVel;
//    float   FOGYawVel;
//    float   FOG_cnt;

//    float   FOGQ0;
//    float   FOGQ1;
//    float   FOGQ2;
//    float   FOGQ3;

    int     IMUStatus;
    int     IMUReading;


    // functions for serial communication
    int     RBOpenPort(int baud);
    int     RBClosePort(void);
    int     RBReadPort(char *_uart_frame, uchar _bytes, uchar _mode);
    int     RBWritePort(const char *_uart_frame, uchar _bytes, uchar _mode);
    int     RBGetReceivedDataByte(char *_data);

private:
    int         isWorking;
    RT_TASK     ReceiveThreadHandler;
    static void RBIMU_ReadThread(void *_arg);


    int     clearBuf(void);
};

#endif // RBIMUSENSOR_H
