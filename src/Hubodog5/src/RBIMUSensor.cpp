#include "RBIMUSensor.h"

#define VectorNav
//#define ADIS


using namespace std;

extern pRBCORE_SHM sharedData;

//float p[4] = {1.0,0.,0.,0.};
//float q[4] = {1.0,0.,0.,0.};
//float Q[4] = {1.0,0.,0.,0.};

int imu_count = 0;

RBIMUSensor::RBIMUSensor()
{
    RS232DeviceHandler = 0;
    isWorking = false;

//    IMUNullFlag = false;
//    IMURoll = IMUPitch = IMUYaw = 0.0;

    IMUReading = false;
    IMUStatus = -1;
}

RBIMUSensor::~RBIMUSensor(){
    isWorking = false;
    usleep(500*1000);
}

int RBIMUSensor::RBOpenPort(int baud){
    int port_fd, ret;//, ret1, ret2;
    struct termios settings;

    char port_name[50];
    int i;
    for(i=0; i<5; i++){
        sprintf(port_name, "/dev/ttyUSB%d",i);
        port_fd = open(port_name, O_RDWR|O_NONBLOCK|O_NOCTTY);
        if(port_fd == -1){
            IMUStatus = IMU_PORT_OPEN_ERROR;
            continue;
        }else{
            fcntl(port_fd, F_SETFL, 0);
            RS232DeviceHandler = port_fd;

            // get current port setting
            tcgetattr(RS232DeviceHandler, &settings);

            //settng the baud rate add error checking
            //ret1 = cfsetispeed(&settings, baud);
            //ret2 = cfsetospeed(&settings, baud);
            //if(ret1==-1 && ret2==-1) return -2;

            //set local mode & enable receiver
            settings.c_cflag        = baud | CS8 | CLOCAL | CREAD;
            settings.c_iflag        = IGNPAR;
            settings.c_oflag        = 0;
            settings.c_lflag        = 0;
            settings.c_cc[VMIN]     = 0;
            settings.c_cc[VTIME]    = 0;

            ret = tcsetattr(RS232DeviceHandler, TCSANOW, &settings);
            if(ret == -1){
                IMUStatus = IMU_OTHER_SET_ERROR;
                return IMU_OTHER_SET_ERROR;
            }

            if(rt_task_create(&ReceiveThreadHandler, "RBIMU_READ_TASK", 0, 99, 0) == 0){
                cpu_set_t aCPU;
                CPU_ZERO(&aCPU);

                // uiuk - very very important
                CPU_SET(2, &aCPU);
                // check cpu load

                if(rt_task_set_affinity(&ReceiveThreadHandler, &aCPU) != 0){
                    FILE_LOG(logWARNING) << "RBIMU: Read thread set affinity CPU failed..";
                }
                if(rt_task_start(&ReceiveThreadHandler, &RBIMU_ReadThread, this) == 0){

                }else{
                    FILE_LOG(logERROR) << "RBIMU: Read thread Creation Error";
                    IMUStatus = IMU_THREAD_CREATE_ERROR;
                    return IMU_THREAD_CREATE_ERROR;
                }
            }else{
                FILE_LOG(logERROR) << "RBIMU: Read thread Creation Error";
                IMUStatus = IMU_THREAD_CREATE_ERROR;
                return IMU_THREAD_CREATE_ERROR;
            }
            break;
        }
    }

    if(i == 5){
        FILE_LOG(logERROR) << "IMU checking port over error";
        IMUStatus = IMU_CHECK_OVER_ERROR;
        return IMU_CHECK_OVER_ERROR;
    }

    FILE_LOG(logSUCCESS) << "IMU Port Open Success [ttyUSB" << i << "]";
    IMUStatus = IMU_PORT_SUCCESS;
    return IMU_PORT_SUCCESS;
}

int RBIMUSensor::RBClosePort(void){
    isWorking = false;
    usleep(100*1000);

    rt_task_delete(&ReceiveThreadHandler);
    return true;
}

int RBIMUSensor::RBReadPort(char *_uart_frame, uchar _bytes, uchar _mode){
    uchar receivedByte = 0;
    uchar index = 0;
    uint loopTime = 0;
    char receivedData[20];
    uchar i;

    if(_mode == 0x00){
        usleep(10000);
        return read(RS232DeviceHandler, _uart_frame, _bytes);
    }else{
        while(receivedByte < _bytes){
            usleep(10000);

            index = read(RS232DeviceHandler, &receivedData[index], 20);
            if(index > 0){
                for(i=0 ; i<index ; i++) _uart_frame[receivedByte+i] = receivedData[i];
                receivedByte += index;
            }

            if(loopTime > 50) return -1;
            else loopTime++;
        }
        return receivedByte;
    }
}

int RBIMUSensor::RBWritePort(const char *_uart_frame, uchar _bytes, uchar _mode){
    char temp[20];
    if(_mode == 0x00) while((RBReadPort(temp, 20, 0x00) != 0));

    return write(RS232DeviceHandler, _uart_frame, _bytes);
}

int RBIMUSensor::clearBuf(void){
    char temp[20];

    while(RBReadPort(temp, 20, 0x00) > 0);
    StoredDataIndex = ReadDataIndex = 0;

    return 1;
}

int RBIMUSensor::RBGetReceivedDataByte(char *_data){
    if(StoredDataIndex%RS232_RECEIVE_DATA_BUFFER_SIZE != ReadDataIndex){
        *_data = ReceivedData[ReadDataIndex];
        ReadDataIndex++;

        ReadDataIndex %= RS232_RECEIVE_DATA_BUFFER_SIZE;
        return 1;
    }
    else return -1;
}

QElapsedTimer timer;
long nanosec;
long nanosec_pre;
int timer_flag = 0;
long dif;
double dif_ms;

long rx_suc;
long rx_err;
double rx_e_ratio;

void RBIMUSensor::RBIMU_ReadThread(void *_arg)
{
    RBIMUSensor *imu = (RBIMUSensor *)_arg;
    imu->isWorking = true;

    int index = 0;
    //uchar tempData[1024];
    uchar tempData_;
    uchar rxData[30];
    uchar crc;
    static int rx_state = 0;

    int data_length = 20;

#ifdef VectorNav
    data_length = 40; // vectornav
#elif ADIS
    data_length = 20; // adis
#endif



    while(imu->isWorking)
    {

        int sleep_time = 30;

#ifdef VectorNav
        sleep_time = 30;
#elif ADIS
        sleep_time = 30;
#endif



//        index = read(imu->RS232DeviceHandler, &tempData_, 1);
        //cout << (int)tempData_ << endl;

        if(rx_state == 0) {
            index = read(imu->RS232DeviceHandler, &tempData_, 1);
            usleep(sleep_time);
            if(index == 1) {
                index = 0;
                if(tempData_ == 0xff) {rxData[0] = 0xff; rx_state = 1;}
                else rx_state = 0;
            }
        }
        else if(rx_state == 1) {
            index = read(imu->RS232DeviceHandler, &tempData_, 1);
            usleep(sleep_time);
            if(index == 1) {
                index = 0;
                if(tempData_ == 0xff) {rxData[1] = 0xff; rx_state = 2;}
                else rx_state = 0;
            }
        }

        for(int i=2; i<data_length+3; i++) {
            if(i == rx_state) {
                index = read(imu->RS232DeviceHandler, &tempData_, 1);
                usleep(sleep_time);
                if(index == 1) {
                    index = 0;
                    rxData[i] = tempData_;
                    rx_state++;
                }
            }
        }

        if(rx_state == data_length+3) {
            crc = 0;
            for(int i=2; i<data_length+2; i++) {
                crc += rxData[i];
            }
            if(rxData[data_length+2] == (uchar)~crc) {
#ifdef VectorNav
                // vectornav
                uint8_t quat1_temp[4],quat2_temp[4],quat3_temp[4],quat4_temp[4];
                uint8_t roll_temp[4], pitch_temp[4], yaw_temp[4];
                uint8_t gx_temp[4], gy_temp[4], gz_temp[4];
                uint8_t ax_temp[4], ay_temp[4], az_temp[4];
                float roll_f, pitch_f, yaw_f, gx_f, gy_f, gz_f, quat1_f,quat2_f,quat3_f,quat4_f, accx_f, accy_f, accz_f;

                for(int i=0; i<4; i++) {
                    quat1_temp[i] = rxData[2+i];
                    quat2_temp[i] = rxData[2+4+i];
                    quat3_temp[i] = rxData[2+8+i];
                    quat4_temp[i] = rxData[2+12+i];
                    gx_temp[i] = rxData[2+16+i];
                    gy_temp[i] = rxData[2+20+i];
                    gz_temp[i] = rxData[2+24+i];
                    ax_temp[i] = rxData[2+28+i];
                    ay_temp[i] = rxData[2+32+i];
                    az_temp[i] = rxData[2+36+i];
                }

//                FILE_LOG(logSUCCESS) << "gx_f : " << (int)rxData[18] << ", " << (int)rxData[18] << ", " << (int)rxData[20] << ", " << (int)rxData[21];

//                memcpy(&roll_f, roll_temp, 4);
//                memcpy(&pitch_f, pitch_temp, 4);
//                memcpy(&yaw_f, yaw_temp, 4);

                memcpy(&quat1_f, quat1_temp, 4);
                memcpy(&quat2_f, quat2_temp, 4);
                memcpy(&quat3_f, quat3_temp, 4);
                memcpy(&quat4_f, quat4_temp, 4);

                memcpy(&gx_f, gx_temp, 4);
                memcpy(&gy_f, gy_temp, 4);
                memcpy(&gz_f, gz_temp, 4);

                memcpy(&accx_f, ax_temp, 4);
                memcpy(&accy_f, ay_temp, 4);
                memcpy(&accz_f, az_temp, 4);

                double q0 = (double)quat4_f;
                double q1 = (double)quat1_f;
                double q2 = (double)quat2_f;
                double q3 = (double)quat3_f;

                sharedData->IMU[0].Q[0] = q0;
                sharedData->IMU[0].Q[1] = q1;
                sharedData->IMU[0].Q[2] = q2;
                sharedData->IMU[0].Q[3] = q3;

                double phi = atan2(2 * (q2*q3 + q0*q1), 1 - 2 * (q1*q1 + q2*q2))*57.2957914;
                double theta = -1 * asin(2 * (q1*q3 - q0*q2))*57.2957914;
                double psi = atan2(2 * (q1*q2 + q0*q3), 1 - 2 * (q2*q2 + q3*q3))*57.2957914;

                sharedData->IMU[0].Roll = phi - sharedData->roll_ofs;
                sharedData->IMU[0].Pitch = -theta - sharedData->pitch_ofs;
                sharedData->IMU[0].Yaw = -psi;

                sharedData->IMU[0].RollVel = gx_f*57.2957914;
                sharedData->IMU[0].PitchVel = -gy_f*57.2957914;
                sharedData->IMU[0].YawVel = -gz_f*57.2957914;

                sharedData->IMU[0].AccX = (double)accx_f;
                sharedData->IMU[0].AccY = (double)accy_f;
                sharedData->IMU[0].AccZ = (double)accz_f;




#elif ADIS
                //FILE_LOG(logSUCCESS);

                 //adis
                if(sharedData->RaisimSimulation == false) {
                    double q0 = (double)((short)((rxData[3] << 8) | rxData[2])) / 30000.0;
                    double q1 = (double)((short)((rxData[5] << 8) | rxData[4])) / 30000.0;
                    double q2 = (double)((short)((rxData[7] << 8) | rxData[6])) / 30000.0;
                    double q3 = (double)((short)((rxData[9] << 8) | rxData[8])) / 30000.0;

                    double phi = atan2(2 * (q2*q3 + q0*q1), 1 - 2 * (q1*q1 + q2*q2))*57.2957914;
                    double theta = -1 * asin(2 * (q1*q3 - q0*q2))*57.2957914;
                    double psi = atan2(2 * (q1*q2 + q0*q3), 1 - 2 * (q2*q2 + q3*q3))*57.2957914;

                    sharedData->IMU[0].Roll = phi - sharedData->roll_ofs;
                    sharedData->IMU[0].Pitch = theta - sharedData->pitch_ofs;
                    sharedData->IMU[0].Yaw = psi;

                    sharedData->IMU[0].Q[0] = q0;
                    sharedData->IMU[0].Q[1] = q1;
                    sharedData->IMU[0].Q[2] = q2;
                    sharedData->IMU[0].Q[3] = q3;

//                    sharedData->IMU[0].Yaw = psi;
//                    sharedData->IMU[0].Pitch = theta;
//                    sharedData->IMU[0].Roll = phi;
                    sharedData->IMU[0].NO_RESPONSE_CNT = 0;

                    double roll_vel = (double)((short)((rxData[11] << 8) | rxData[10])) / 100.0;
                    double pitch_vel = (double)((short)((rxData[13] << 8) | rxData[12])) / 100.0;
                    double yaw_vel = (double)((short)((rxData[15] << 8) | rxData[14])) / 100.0;
                    //char imu_limit = rxData[16];
                    sharedData->IMU[0].RollVel = roll_vel;
                    sharedData->IMU[0].PitchVel = pitch_vel;
                    sharedData->IMU[0].YawVel = yaw_vel;
//                    if(imu_limit == 1){
//                        cout<<"IMU limit exceed...!!!"<<endl;
//                    }

//                    double acc_x = (double)((short)((rxData[18] << 8) | rxData[17])) *9.80665 / 10000.0;
//                    double acc_y = (double)((short)((rxData[20] << 8) | rxData[19])) *9.80665 / 10000.0;
//                    double acc_z = (double)((short)((rxData[22] << 8) | rxData[21])) *9.80665 / 10000.0;

//                    double acc_x = (double)((short)((rxData[17] << 8) | rxData[16])) / 20.0 * 9.80655/10000.0;
//                    double acc_y = (double)((short)((rxData[19] << 8) | rxData[18])) / 20.0 * 9.80655/10000.0;
//                    double acc_z = (double)((short)((rxData[21] << 8) | rxData[20])) / 20.0 * 9.80655/10000.0;


//                    sharedData->IMU[0].AccX = acc_x;//- sharedData->roll_ofs;
//                    sharedData->IMU[0].AccY = acc_y;// - sharedData->pitch_ofs;
//                    sharedData->IMU[0].AccZ = acc_z;

                    double q0_AHRS = (double)((short)((rxData[15] << 8) | rxData[14])) / 30000.0;
                    double q1_AHRS = (double)((short)((rxData[17] << 8) | rxData[16])) / 30000.0;
                    double q2_AHRS = (double)((short)((rxData[19] << 8) | rxData[18])) / 30000.0;
                    double q3_AHRS = (double)((short)((rxData[21] << 8) | rxData[20])) / 30000.0;

                    double phi_AHRS = atan2(2 * (q2_AHRS*q3_AHRS + q0_AHRS*q1_AHRS), 1 - 2 * (q1_AHRS*q1_AHRS + q2_AHRS*q2_AHRS))*57.2957914;
                    double theta_AHRS = -1 * asin(2 * (q1_AHRS*q3_AHRS - q0_AHRS*q2_AHRS))*57.2957914;
                    double psi_AHRS = atan2(2 * (q1_AHRS*q2_AHRS + q0_AHRS*q3_AHRS), 1 - 2 * (q2_AHRS*q2_AHRS + q3_AHRS*q3_AHRS))*57.2957914;

                    sharedData->IMU[0].Roll = phi_AHRS - sharedData->roll_ofs;
                    sharedData->IMU[0].Pitch = theta_AHRS - sharedData->pitch_ofs;
                    sharedData->IMU[0].Yaw = psi_AHRS;

//                    sharedData->IMU[0].Roll_AHRS = phi_AHRS - sharedData->roll_ofs;
//                    sharedData->IMU[0].Pitch_AHRS = theta_AHRS - sharedData->pitch_ofs;
//                    sharedData->IMU[0].Yaw_AHRS = psi_AHRS;

//                    sharedData->IMU[0].AccX = phi - sharedData->roll_ofs;
//                    sharedData->IMU[0].AccY = theta - sharedData->pitch_ofs;
//                    sharedData->IMU[0].AccZ = psi;

//                    sharedData->comcon_variable_plot[0] = sharedData->IMU[0].Roll;
//                    sharedData->comcon_variable_plot[1] = sharedData->IMU[0].Pitch;
//                    sharedData->comcon_variable_plot[2] = sharedData->IMU[0].Roll_AHRS;
//                    sharedData->comcon_variable_plot[3] = sharedData->IMU[0].Pitch_AHRS;
//                    sharedData->comcon_variable_plot[4] = 0;
//                    sharedData->comcon_variable_plot[5] = 0;

                }


//                if(!timer_flag){
//                    timer.start();
//                    timer_flag = 1;
//                }
//                nanosec_pre = nanosec;
//                nanosec = timer.nsecsElapsed();

//                dif = nanosec - nanosec_pre;
//                dif_ms = (double)dif / 1000000.;

//                if(dif_ms < 2) {
//                    FILE_LOG(logSUCCESS) << "dif : " << dif_ms << ",  " << (int)rxData[0] << ", "  << (int)rxData[1]  << ", " << (int)rxData[2]  << ", " << (int)rxData[3]  << ", "
//                                         << (int)rxData[4] << ", "  << (int)rxData[5]  << ", " << (int)rxData[6]  << ", " << (int)rxData[7]  << ", "
//                                         << (int)rxData[8] << ", "  << (int)rxData[9]  << ", " << (int)rxData[10] << ", " << (int)rxData[11] << ", "
//                                         << (int)rxData[12] << ", " << (int)rxData[13] << ", " << (int)rxData[14] << ", " << (int)rxData[15];
//                }
//                else {
//                    FILE_LOG(logWARNING) << "dif : " << dif_ms << ",  " << (int)rxData[0] << ", "  << (int)rxData[1]  << ", " << (int)rxData[2]  << ", " << (int)rxData[3]  << ", "
//                                         << (int)rxData[4] << ", "  << (int)rxData[5]  << ", " << (int)rxData[6]  << ", " << (int)rxData[7]  << ", "
//                                         << (int)rxData[8] << ", "  << (int)rxData[9]  << ", " << (int)rxData[10] << ", " << (int)rxData[11] << ", "
//                                         << (int)rxData[12] << ", " << (int)rxData[13] << ", " << (int)rxData[14] << ", " << (int)rxData[15];
//                }
#endif
            }
            else {
                FILE_LOG(logERROR) << "crc not match" << ", crc_calc : " << (int)((uchar)~crc) << ",  crc_get : " << (int)rxData[23];
            }

            rx_state = 0;

        }
    }
}
