#include "mainwindow.h"
#include "ui_mainwindow.h"

#include "RBLog.h"
#include "JointInformation.h"
#include "RBLANComm.h"
#include "index_notation.h"

#include "RBSharedMemory.h"
#include <QTimer>
#include <QPen>

#define J_A       0
#define J_B       1
#define J_X       2
#define J_Y       3
#define J_LB      4
#define J_RB      5
#define J_BACK    6
#define J_START   7
#define J_LOGI    8
#define J_LJOY    9
#define J_RJOY    10

#define J_NOACT       0
#define J_READYPOS    1
#define J_GROUNDPOS   2
#define J_MOTIONSTOP  3
#define J_BACKFLIP    4
#define J_IMUNULLING  5
#define J_FALLRECOVER 6

#define J_WALKREADY   11
#define J_WALKSTART   12
#define J_WALKSTOP    13
#define J_IMURESET    14

#define J_TROTWALKING     21
#define J_TROTWALKING_S   22
#define J_F_TROTWALKING   23
#define J_PRONKING        24

const double R2Df = 57.2957802f;
const double D2Rf = 0.0174533f;

extern int MY_CONTROL_FB_GAIN[12];
extern int MY_CONTROL_FF_GAIN[12];

extern pRBCORE_SHM          sharedData;
extern RBMotorController    _DEV_MC[MAX_MC];
extern RBIMUSensor         _DEV_IMU;
extern int                  _NO_OF_MC;
extern RBLANComm   *lanHandler;
#define rad2deg 57.2958
using namespace std;

int Jog_state=0;
int Jog_Up_state=0, Jog_Down_state=0;
int Jog_speed_state = 0;
int Jog_mode = 1;
double Jog_JointMov = 0;
const double Jog_JointVel = 0.2*D2Rf;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    InitTable(ui->TW_0, JointNameList, NO_OF_JOINTSJ);
    lastSelected = J0;
    select_working = false;
    ChangeSelectedJoint();
    err_clr_start = false;

    model = new ModelDialog(ui->FRAME_MODEL);
    model->setWindowFlags(Qt::Widget);
    ui->FRAME_MODEL->setFixedSize(model->size());
    model->move(0, 0);

    GraphInitialize();

    displayTimer = new QTimer();
    connect(displayTimer, SIGNAL(timeout()), this, SLOT(DisplayUpdate_Main()));
    displayTimer->start(10);

    graphTimer = new QTimer();
    connect(graphTimer, SIGNAL(timeout()), this, SLOT(DispalyUpdate_BasicTest()));
    //connect(graphTimer, SIGNAL(timeout()), this, SLOT(DispalyUpdate_MPC()));
    graphTimer->start(2);


    for(int i=0;i<12;i++){oldB[i] = 0;}
    joy = new RBJoystick();
    JoyCommand = DemoMotion;

    if(joy->connection==false)
    {
        FILE_LOG(logINFO) << "JOYSTICK_ENABLE";
        joy->ConnectJoy(ui->LE_JOYNAME->text());
    }
//    FILE_LOG(logINFO) << "CMD: DAEMON_SENSOR_IMU_ENABLE";
//    _DEV_IMU.RBOpenPort(B460800);


}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::BasicSetup(){
    if(ui->CB_isJointFilter->isChecked()){
        sharedData->isJointFilter = true;
        sharedData->frq_JointFilter = ui->LE_JointFilter->text().toDouble();
        FILE_LOG(logSUCCESS) << "Joint Filter ON. Cutoff : " << sharedData->frq_JointFilter << "Hz";
    }
    else{sharedData->isJointFilter = false;}

    sharedData->z0 = ui->LE_z0->text().toDouble();
    sharedData->x0 = ui->LE_x0->text().toDouble();
}

void MainWindow::LegControlSetup(){
    sharedData->Kp[0] = ui->Kp_x->text().toDouble();
    sharedData->Kp[1] = ui->Kp_y->text().toDouble();
    sharedData->Kp[2] = ui->Kp_z->text().toDouble();

    sharedData->Kd[0] = ui->Kd_x->text().toDouble();
    sharedData->Kd[1] = ui->Kd_y->text().toDouble();
    sharedData->Kd[2] = ui->Kd_z->text().toDouble();

    sharedData->Kp_j[0] = ui->Kp_r->text().toDouble();
    sharedData->Kp_j[1] = ui->Kp_p->text().toDouble();
    sharedData->Kp_j[2] = ui->Kp_k->text().toDouble();

    sharedData->Kd_j[0] = ui->Kd_r->text().toDouble();
    sharedData->Kd_j[1] = ui->Kd_p->text().toDouble();
    sharedData->Kd_j[2] = ui->Kd_k->text().toDouble();

    sharedData->Cmax = ui->LE_Cmax->text().toDouble();


}

void MainWindow::MPCGainUpdate()
{

}

double calc_joy(double in)
{
    double thres = 1000;
    double MaxA = 30000;
    double td;
    if(in>thres)
    {
        td = min(1.0,(in-thres)/(MaxA-thres));
    }
    else if(in<-thres)
    {
        td = max(-1.0,(in+thres)/(MaxA-thres));
    }
    else
    {
        td = 0;
    }
    return td;

}

int cnt_joy;
bool imu_null_flag = 0;
bool imu_null_flag_pre = 0;
void MainWindow::DisplayUpdate_Main(){

    int row;
    QTableWidget *tw;
    QString str;


    if(lanHandler->ConnectionStatus){
        ui->LE_CAN_CONNECT_STATUS->setStyleSheet("background-color:green");
    }else{
        ui->LE_CAN_CONNECT_STATUS->setStyleSheet("background-color:red");
    }


    // current ref status display ------
    if(_DEV_MC[0].ReferenceOutEnable == 1 && _DEV_MC[1].ReferenceOutEnable == 1 && _DEV_MC[2].ReferenceOutEnable == 1 &&
        _DEV_MC[3].ReferenceOutEnable == 1 && _DEV_MC[4].ReferenceOutEnable == 1 && _DEV_MC[5].ReferenceOutEnable == 1){
        sharedData->REF_Enabled = true;
    }else{
        sharedData->REF_Enabled = false;
    }

    if(sharedData->REF_Enabled == true){
        ui->LE_REF_STATUS->setStyleSheet("background-color:green");
    }else{
        ui->LE_REF_STATUS->setStyleSheet("background-color:red");
    }

    // current reference display ------
//    ui->LE_JNT0->setText(QString().sprintf("%.2f", _DEV_MC[0].MoveJoints.RefAngleCurrent));
//    ui->LE_JNT1->setText(QString().sprintf("%.2f", _DEV_MC[1].MoveJoints.RefAngleCurrent));
//    ui->LE_JNT2->setText(QString().sprintf("%.2f", _DEV_MC[2].MoveJoints.RefAngleCurrent));
//    ui->LE_JNT3->setText(QString().sprintf("%.2f", _DEV_MC[3].MoveJoints.RefAngleCurrent));
//    ui->LE_JNT4->setText(QString().sprintf("%.2f", _DEV_MC[4].MoveJoints.RefAngleCurrent));
//    ui->LE_JNT5->setText(QString().sprintf("%.2f", _DEV_MC[5].MoveJoints.RefAngleCurrent));
//    ui->LE_JNT6->setText(QString().sprintf("%.2f", _DEV_MC[6].MoveJoints.RefAngleCurrent));
//    ui->LE_JNT7->setText(QString().sprintf("%.2f", _DEV_MC[7].MoveJoints.RefAngleCurrent));
//    ui->LE_JNT8->setText(QString().sprintf("%.2f", _DEV_MC[8].MoveJoints.RefAngleCurrent));
//    ui->LE_JNT9->setText(QString().sprintf("%.2f", _DEV_MC[9].MoveJoints.RefAngleCurrent));
//    ui->LE_JNT10->setText(QString().sprintf("%.2f", _DEV_MC[10].MoveJoints.RefAngleCurrent));
//    ui->LE_JNT11->setText(QString().sprintf("%.2f", _DEV_MC[11].MoveJoints.RefAngleCurrent));
    ui->LE_JNT0->setText(QString().sprintf("%.2f",  R2Df * sharedData->joint_info_ref[0]));
    ui->LE_JNT1->setText(QString().sprintf("%.2f",  R2Df * sharedData->joint_info_ref[1]));
    ui->LE_JNT2->setText(QString().sprintf("%.2f",  R2Df * sharedData->joint_info_ref[2]));
    ui->LE_JNT3->setText(QString().sprintf("%.2f",  R2Df * sharedData->joint_info_ref[3]));
    ui->LE_JNT4->setText(QString().sprintf("%.2f",  R2Df * sharedData->joint_info_ref[4]));
    ui->LE_JNT5->setText(QString().sprintf("%.2f",  R2Df * sharedData->joint_info_ref[5]));
    ui->LE_JNT6->setText(QString().sprintf("%.2f",  R2Df * sharedData->joint_info_ref[6]));
    ui->LE_JNT7->setText(QString().sprintf("%.2f",  R2Df * sharedData->joint_info_ref[7]));
    ui->LE_JNT8->setText(QString().sprintf("%.2f",  R2Df * sharedData->joint_info_ref[8]));
    ui->LE_JNT9->setText(QString().sprintf("%.2f",  R2Df * sharedData->joint_info_ref[9]));
    ui->LE_JNT10->setText(QString().sprintf("%.2f", R2Df * sharedData->joint_info_ref[10]));
    ui->LE_JNT11->setText(QString().sprintf("%.2f", R2Df * sharedData->joint_info_ref[11]));

    ui->LE_POWER_VIN->setText(QString().sprintf("%.2f", sharedData->Vin));
    ui->LE_POWER_IIN->setText(QString().sprintf("%.1f", sharedData->Iin));




    // current encoder display ------
//    ui->LE_JNT0_2->setText(QString().sprintf("%.2f", _DEV_MC[0].CurrentPosition));
//    ui->LE_JNT1_2->setText(QString().sprintf("%.2f", _DEV_MC[1].CurrentPosition));
//    ui->LE_JNT2_2->setText(QString().sprintf("%.2f", _DEV_MC[2].CurrentPosition));
//    ui->LE_JNT3_2->setText(QString().sprintf("%.2f", _DEV_MC[3].CurrentPosition));
//    ui->LE_JNT4_2->setText(QString().sprintf("%.2f", _DEV_MC[4].CurrentPosition));
//    ui->LE_JNT5_2->setText(QString().sprintf("%.2f", _DEV_MC[5].CurrentPosition));
//    ui->LE_JNT6_2->setText(QString().sprintf("%.2f", _DEV_MC[6].CurrentPosition));
//    ui->LE_JNT7_2->setText(QString().sprintf("%.2f", _DEV_MC[7].CurrentPosition));
//    ui->LE_JNT8_2->setText(QString().sprintf("%.2f", _DEV_MC[8].CurrentPosition));
//    ui->LE_JNT9_2->setText(QString().sprintf("%.2f", _DEV_MC[9].CurrentPosition));
//    ui->LE_JNT10_2->setText(QString().sprintf("%.2f", _DEV_MC[10].CurrentPosition));
//    ui->LE_JNT11_2->setText(QString().sprintf("%.2f", _DEV_MC[11].CurrentPosition));

    ui->LE_JNT0_2->setText(QString().sprintf("%.2f",  R2Df * sharedData->joint_info[0]));
    ui->LE_JNT1_2->setText(QString().sprintf("%.2f",  R2Df * sharedData->joint_info[1]));
    ui->LE_JNT2_2->setText(QString().sprintf("%.2f",  R2Df * sharedData->joint_info[2]));
    ui->LE_JNT3_2->setText(QString().sprintf("%.2f",  R2Df * sharedData->joint_info[3]));
    ui->LE_JNT4_2->setText(QString().sprintf("%.2f",  R2Df * sharedData->joint_info[4]));
    ui->LE_JNT5_2->setText(QString().sprintf("%.2f",  R2Df * sharedData->joint_info[5]));
    ui->LE_JNT6_2->setText(QString().sprintf("%.2f",  R2Df * sharedData->joint_info[6]));
    ui->LE_JNT7_2->setText(QString().sprintf("%.2f",  R2Df * sharedData->joint_info[7]));
    ui->LE_JNT8_2->setText(QString().sprintf("%.2f",  R2Df * sharedData->joint_info[8]));
    ui->LE_JNT9_2->setText(QString().sprintf("%.2f",  R2Df * sharedData->joint_info[9]));
    ui->LE_JNT10_2->setText(QString().sprintf("%.2f", R2Df * sharedData->joint_info[10]));
    ui->LE_JNT11_2->setText(QString().sprintf("%.2f", R2Df * sharedData->joint_info[11]));


    // current current display ------
    ui->LE_JNT0_3->setText(QString().sprintf("%d", _DEV_MC[0].CurrentReference_mA*_DEV_MC[0].TPC));
    ui->LE_JNT1_3->setText(QString().sprintf("%d", _DEV_MC[1].CurrentReference_mA*_DEV_MC[1].TPC));
    ui->LE_JNT2_3->setText(QString().sprintf("%d", _DEV_MC[2].CurrentReference_mA*_DEV_MC[2].TPC));
    ui->LE_JNT3_3->setText(QString().sprintf("%d", _DEV_MC[3].CurrentReference_mA*_DEV_MC[3].TPC));
    ui->LE_JNT4_3->setText(QString().sprintf("%d", _DEV_MC[4].CurrentReference_mA*_DEV_MC[4].TPC));
    ui->LE_JNT5_3->setText(QString().sprintf("%d", _DEV_MC[5].CurrentReference_mA*_DEV_MC[5].TPC));
    ui->LE_JNT6_3->setText(QString().sprintf("%d", _DEV_MC[6].CurrentReference_mA*_DEV_MC[6].TPC));
    ui->LE_JNT7_3->setText(QString().sprintf("%d", _DEV_MC[7].CurrentReference_mA*_DEV_MC[7].TPC));
    ui->LE_JNT8_3->setText(QString().sprintf("%d", _DEV_MC[8].CurrentReference_mA*_DEV_MC[8].TPC));
    ui->LE_JNT9_3->setText(QString().sprintf("%d", _DEV_MC[9].CurrentReference_mA*_DEV_MC[9].TPC));
    ui->LE_JNT10_3->setText(QString().sprintf("%d", _DEV_MC[10].CurrentReference_mA*_DEV_MC[10].TPC));
    ui->LE_JNT11_3->setText(QString().sprintf("%d", _DEV_MC[11].CurrentReference_mA*_DEV_MC[11].TPC));


    // current sensing disply -----
    ui->LE_CURSEN_0->setText(QString().sprintf("%d", _DEV_MC[0].MeasuredCurrent));
    ui->LE_CURSEN_1->setText(QString().sprintf("%d", _DEV_MC[1].MeasuredCurrent));
    ui->LE_CURSEN_2->setText(QString().sprintf("%d", _DEV_MC[2].MeasuredCurrent));
    ui->LE_CURSEN_3->setText(QString().sprintf("%d", _DEV_MC[3].MeasuredCurrent));
    ui->LE_CURSEN_4->setText(QString().sprintf("%d", _DEV_MC[4].MeasuredCurrent));
    ui->LE_CURSEN_5->setText(QString().sprintf("%d", _DEV_MC[5].MeasuredCurrent));
    ui->LE_CURSEN_6->setText(QString().sprintf("%d", _DEV_MC[6].MeasuredCurrent));
    ui->LE_CURSEN_7->setText(QString().sprintf("%d", _DEV_MC[7].MeasuredCurrent));
    ui->LE_CURSEN_8->setText(QString().sprintf("%d", _DEV_MC[8].MeasuredCurrent));
    ui->LE_CURSEN_9->setText(QString().sprintf("%d", _DEV_MC[9].MeasuredCurrent));
    ui->LE_CURSEN_10->setText(QString().sprintf("%d", _DEV_MC[10].MeasuredCurrent));
    ui->LE_CURSEN_11->setText(QString().sprintf("%d", _DEV_MC[11].MeasuredCurrent));

    // PCON CCON gain display -----
    ui->PCON_HRR->setText(QString().sprintf("%d", sharedData->PCON[0]));
    ui->PCON_HRP->setText(QString().sprintf("%d", sharedData->PCON[1]));
    ui->PCON_HRK->setText(QString().sprintf("%d", sharedData->PCON[2]));
    ui->PCON_HLR->setText(QString().sprintf("%d", sharedData->PCON[3]));
    ui->PCON_HLP->setText(QString().sprintf("%d", sharedData->PCON[4]));
    ui->PCON_HLK->setText(QString().sprintf("%d", sharedData->PCON[5]));
    ui->PCON_FRR->setText(QString().sprintf("%d", sharedData->PCON[6]));
    ui->PCON_FRP->setText(QString().sprintf("%d", sharedData->PCON[7]));
    ui->PCON_FRK->setText(QString().sprintf("%d", sharedData->PCON[8]));
    ui->PCON_FLR->setText(QString().sprintf("%d", sharedData->PCON[9]));
    ui->PCON_FLP->setText(QString().sprintf("%d", sharedData->PCON[10]));
    ui->PCON_FLK->setText(QString().sprintf("%d", sharedData->PCON[11]));

    ui->CCON_HRR->setText(QString().sprintf("%d", sharedData->CCON[0]));
    ui->CCON_HRP->setText(QString().sprintf("%d", sharedData->CCON[1]));
    ui->CCON_HRK->setText(QString().sprintf("%d", sharedData->CCON[2]));
    ui->CCON_HLR->setText(QString().sprintf("%d", sharedData->CCON[3]));
    ui->CCON_HLP->setText(QString().sprintf("%d", sharedData->CCON[4]));
    ui->CCON_HLK->setText(QString().sprintf("%d", sharedData->CCON[5]));
    ui->CCON_FRR->setText(QString().sprintf("%d", sharedData->CCON[6]));
    ui->CCON_FRP->setText(QString().sprintf("%d", sharedData->CCON[7]));
    ui->CCON_FRK->setText(QString().sprintf("%d", sharedData->CCON[8]));
    ui->CCON_FLR->setText(QString().sprintf("%d", sharedData->CCON[9]));
    ui->CCON_FLP->setText(QString().sprintf("%d", sharedData->CCON[10]));
    ui->CCON_FLK->setText(QString().sprintf("%d", sharedData->CCON[11]));

    // error clear
    if(err_clr_start == true){
        if(err_clr_cnt == 0){
            _DEV_MC[err_clr_jnt_id].RBBoard_ReferenceOutEnable(false);
            // encoder enable
            _DEV_MC[err_clr_jnt_id].RBBoard_RequestEncoder(1);
        }else if(err_clr_cnt == 3){
            // sleep for encoder read
            // refresh reference to encoder value
            _DEV_MC[err_clr_jnt_id].MoveJoints.RefAngleCurrent = _DEV_MC[err_clr_jnt_id].CurrentPosition - _DEV_MC[err_clr_jnt_id].homeJointOffset;
            // error clear
            _DEV_MC[err_clr_jnt_id].RBJoint_ClearErrorFlag(err_clr_jnt_ch+1);
            // FET on & CTRL on
            _DEV_MC[err_clr_jnt_id].RBJoint_EnableFETDriver(err_clr_jnt_ch+1, true);
        }else if(err_clr_cnt == 4){
            _DEV_MC[err_clr_jnt_id].RBJoint_EnableFeedbackControl(err_clr_jnt_ch+1, true);
        }else if(err_clr_cnt == 5){
            // wait for settling
            // reference out enable
            _DEV_MC[err_clr_jnt_id].RBBoard_ReferenceOutEnable(true);
            err_clr_start = false;
        }
        err_clr_cnt++;
    }

    // current board status display ------
    for(int i=0; i<NO_OF_JOINTSJ; i++){
        row = i;
        tw = ui->TW_0;

        mSTAT stat = _DEV_MC[i].CurrentStatus;//
        str = "";
        if(_DEV_MC[i].ConnectionStatus) str += "C  ";
        else                                                str += "N  ";
        if(stat.b.FET == 1) str += "H/";
        else                str += "-/";
        if(stat.b.RUN == 1) str += "R/";
        else                str += "-/";
        str += QString().sprintf("%d", stat.b.INIT);
        tw->item(row, 0)->setText(str);
        if(stat.b.RUN == 1 && stat.b.INIT == 1){
            tw->item(row, 0)->setBackgroundColor(QColor(100, 255, 100));    // green
        }else if(stat.b.RUN == 1 && stat.b.INIT == 0){
            tw->item(row, 0)->setBackgroundColor(QColor(255, 255, 100));    // yellow
        }else{
            tw->item(row, 0)->setBackgroundColor(QColor(255, 100, 100));    // red
        }


        if(stat.b.MT_ERR == 1){
            ui->BTN_FIND_HOME->setDisabled(true);
            std::cout<<"##################### PVL ERROR ##################### at "<<i<<std::endl;
        }


        str = "";
        if(stat.b.JAM == 1) str += "JAM ";
        if(stat.b.CUR == 1) str += "CUR ";
        if(stat.b.BIG == 1) str += "BIG ";
        if(stat.b.INP == 1) str += "INP ";
        if(stat.b.FLT == 1) str += "FLT ";
        if(stat.b.TMP == 1) str += "TMP ";
        if(stat.b.PS1 == 1) str += "PS1 ";
        if(stat.b.PS2 == 1) str += "PS2 ";
        if(stat.b.NON_CTR == 1) str += "NON ";
        if(str == ""){
            str = "-";
            tw->item(row, 1)->setBackgroundColor(QColor(255, 255, 255));
        }else{
            tw->item(row, 1)->setBackgroundColor(QColor(255, 100, 100));
        }
        tw->item(row, 1)->setText(str);


        int board_temperature = _DEV_MC[i].BoardTemperature;
        int motor_temperature = _DEV_MC[i].MotorTemperature;
        str ="";
        str += QString().sprintf("%d", board_temperature);
        str +="/";
        str += QString().sprintf("%d", motor_temperature);

        tw->item(row, 2)->setText(str);
//        if(board_temperature > 60 || motor_temperature > 60)
        if(board_temperature > 60)
            tw->item(row, 2)->setBackgroundColor(QColor(255, 100, 100));
        else
            tw->item(row, 2)->setBackgroundColor(QColor(255, 255, 255));
    }

    //IMU update
//    ui->LE_SENSOR_CIMU_ROLL->setText(str.sprintf("%.3f", sharedData->IMU[1].Roll));
//    ui->LE_SENSOR_CIMU_PITCH->setText(str.sprintf("%.3f", sharedData->IMU[1].Pitch));
//    ui->LE_SENSOR_CIMU_YAW->setText(str.sprintf("%.2f", sharedData->IMU[1].Yaw));
//    ui->LE_SENSOR_CIMU_ROLL_VEL->setText(str.sprintf("%.3f", sharedData->IMU[1].RollVel));
//    ui->LE_SENSOR_CIMU_PITCH_VEL->setText(str.sprintf("%.3f", sharedData->IMU[1].PitchVel));
//    ui->LE_SENSOR_CIMU_ROLL_ACC->setText(str.sprintf("%.3f", sharedData->IMU[1].AccX));
//    ui->LE_SENSOR_CIMU_PITCH_ACC->setText(str.sprintf("%.3f", sharedData->IMU[1].AccY));
//    ui->LE_SENSOR_CIMU_YAW_VEL->setText(str.sprintf("%.2f", sharedData->IMU[1].YawVel));
//    ui->LE_SENSOR_CIMU_YAW_ACC->setText(str.sprintf("%.3f", sharedData->IMU[1].AccZ));
    ui->LE_SENSOR_CIMU_ROLL->setText(str.sprintf("%.3f", sharedData->IMU[0].Roll));
    ui->LE_SENSOR_CIMU_PITCH->setText(str.sprintf("%.3f", sharedData->IMU[0].Pitch));
    ui->LE_SENSOR_CIMU_YAW->setText(str.sprintf("%.2f", sharedData->IMU[0].Yaw));
    ui->LE_SENSOR_CIMU_ROLL_VEL->setText(str.sprintf("%.3f", sharedData->IMU[0].RollVel));
    ui->LE_SENSOR_CIMU_PITCH_VEL->setText(str.sprintf("%.3f", sharedData->IMU[0].PitchVel));
    ui->LE_SENSOR_CIMU_ROLL_ACC->setText(str.sprintf("%.3f", sharedData->IMU[0].AccX));
    ui->LE_SENSOR_CIMU_PITCH_ACC->setText(str.sprintf("%.3f", sharedData->IMU[0].AccY));
    ui->LE_SENSOR_CIMU_YAW_VEL->setText(str.sprintf("%.2f", sharedData->IMU[0].YawVel));
    ui->LE_SENSOR_CIMU_YAW_ACC->setText(str.sprintf("%.3f", sharedData->IMU[0].AccZ));
    ui->LE_SENSOR_CIMU_ROLL_AHRS->setText(str.sprintf("%.3f", sharedData->IMU[0].Roll_AHRS));
    ui->LE_SENSOR_CIMU_PITCH_AHRS->setText(str.sprintf("%.3f", sharedData->IMU[0].Pitch_AHRS));
    ui->LE_SENSOR_CIMU_YAW_AHRS->setText(str.sprintf("%.3f", sharedData->IMU[0].Yaw_AHRS));
    ui->LE_NRC->setText(str.sprintf("%d", sharedData->IMU[0].NO_RESPONSE_CNT));


    if(ui->RBTN_JOINT_0->isChecked())  sharedData->jog_joint[0] = 1;
    else sharedData->jog_joint[0] = 0;
    if(ui->RBTN_JOINT_1->isChecked())  sharedData->jog_joint[1] = 1;
    else sharedData->jog_joint[1] = 0;
    if(ui->RBTN_JOINT_2->isChecked())  sharedData->jog_joint[2] = 1;
    else sharedData->jog_joint[2] = 0;
    if(ui->RBTN_JOINT_3->isChecked())  sharedData->jog_joint[3] = 1;
    else sharedData->jog_joint[3] = 0;
    if(ui->RBTN_JOINT_4->isChecked())  sharedData->jog_joint[4] = 1;
    else sharedData->jog_joint[4] = 0;
    if(ui->RBTN_JOINT_5->isChecked())  sharedData->jog_joint[5] = 1;
    else sharedData->jog_joint[5] = 0;
    if(ui->RBTN_JOINT_6->isChecked())  sharedData->jog_joint[6] = 1;
    else sharedData->jog_joint[6] = 0;
    if(ui->RBTN_JOINT_7->isChecked())  sharedData->jog_joint[7] = 1;
    else sharedData->jog_joint[7] = 0;
    if(ui->RBTN_JOINT_8->isChecked())  sharedData->jog_joint[8] = 1;
    else sharedData->jog_joint[8] = 0;
    if(ui->RBTN_JOINT_9->isChecked())  sharedData->jog_joint[9] = 1;
    else sharedData->jog_joint[9] = 0;
    if(ui->RBTN_JOINT_10->isChecked())  sharedData->jog_joint[10] = 1;
    else sharedData->jog_joint[10] = 0;
    if(ui->RBTN_JOINT_11->isChecked())  sharedData->jog_joint[11] = 1;
    else sharedData->jog_joint[11] = 0;

    if(Jog_mode == 0) {
        ui->BTN_JOG_MODE_1->setStyleSheet("QPushButton{background-color:brown}");
        ui->BTN_JOG_MODE_2->setStyleSheet("QPushButton{background-color:white}");
    }
    else {
        ui->BTN_JOG_MODE_1->setStyleSheet("QPushButton{background-color:white}");
        ui->BTN_JOG_MODE_2->setStyleSheet("QPushButton{background-color:brown}");
    }

    if(Jog_speed_state == 0){
        ui->BTN_JOG_SPEED_1->setStyleSheet("QPushButton{background-color:brown}");
        ui->BTN_JOG_SPEED_2->setStyleSheet("QPushButton{background-color:white}");
        ui->BTN_JOG_SPEED_3->setStyleSheet("QPushButton{background-color:white}");
        ui->BTN_JOG_SPEED_4->setStyleSheet("QPushButton{background-color:white}");
        Jog_JointMov = Jog_JointVel/10;
    }
    else if(Jog_speed_state == 1) {
        ui->BTN_JOG_SPEED_1->setStyleSheet("QPushButton{background-color:white}");
        ui->BTN_JOG_SPEED_2->setStyleSheet("QPushButton{background-color:brown}");
        ui->BTN_JOG_SPEED_3->setStyleSheet("QPushButton{background-color:white}");
        ui->BTN_JOG_SPEED_4->setStyleSheet("QPushButton{background-color:white}");
        Jog_JointMov = Jog_JointVel/4;
    }
    else if(Jog_speed_state == 2) {
        ui->BTN_JOG_SPEED_1->setStyleSheet("QPushButton{background-color:white}");
        ui->BTN_JOG_SPEED_2->setStyleSheet("QPushButton{background-color:white}");
        ui->BTN_JOG_SPEED_3->setStyleSheet("QPushButton{background-color:brown}");
        ui->BTN_JOG_SPEED_4->setStyleSheet("QPushButton{background-color:white}");
        Jog_JointMov = Jog_JointVel/2;
    }
    else if(Jog_speed_state == 3) {
        ui->BTN_JOG_SPEED_1->setStyleSheet("QPushButton{background-color:white}");
        ui->BTN_JOG_SPEED_2->setStyleSheet("QPushButton{background-color:white}");
        ui->BTN_JOG_SPEED_3->setStyleSheet("QPushButton{background-color:white}");
        ui->BTN_JOG_SPEED_4->setStyleSheet("QPushButton{background-color:brown}");
        Jog_JointMov = Jog_JointVel;
    }

    if(sharedData->contact_flag_test[HR]) ui->LE_CONTACT_HR->setStyleSheet("background-color:green");
    else ui->LE_CONTACT_HR->setStyleSheet("background-color:red");
    if(sharedData->contact_flag_test[HL]) ui->LE_CONTACT_HL->setStyleSheet("background-color:green");
    else ui->LE_CONTACT_HL->setStyleSheet("background-color:red");
    if(sharedData->contact_flag_test[FR]) ui->LE_CONTACT_FR->setStyleSheet("background-color:green");
    else ui->LE_CONTACT_FR->setStyleSheet("background-color:red");
    if(sharedData->contact_flag_test[FL]) ui->LE_CONTACT_FL->setStyleSheet("background-color:green");
    else ui->LE_CONTACT_FL->setStyleSheet("background-color:red");

}

void MainWindow::GraphInitialize(){
    QPen myPen, dotPen, filterPen;
    myPen.setWidthF(1);
    filterPen.setStyle(Qt::DotLine);
    filterPen.setWidth(1);
    dotPen.setStyle(Qt::DotLine);
    dotPen.setWidth(20);
    dotPen.setWidthF(2);
    dotPen.setColor(Qt::gray);

    QSharedPointer<QCPAxisTickerTime> timeTicker(new QCPAxisTickerTime);
    timeTicker->setTimeFormat("%m:%s");

    ui->plot_pfoot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP :: iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->plot_pcom->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP :: iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->plot_joint->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP :: iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->plot_imu->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP :: iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->plot_comcon->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP :: iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->plot_fcontact->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP :: iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);

    // foot position
    for(int i=0; i<6; i++)
       ui->plot_pfoot->addGraph();
    myPen.setColor(Qt::red);
    dotPen.setColor(Qt::red);
    ui->plot_pfoot->graph(0)->setPen(myPen);
    ui->plot_pfoot->graph(3)->setPen(dotPen);
    myPen.setColor(Qt::blue);
    dotPen.setColor(Qt::blue);
    ui->plot_pfoot->graph(1)->setPen(myPen);
    ui->plot_pfoot->graph(4)->setPen(dotPen);
    myPen.setColor(Qt::green);
    dotPen.setColor(Qt::green);
    ui->plot_pfoot->graph(2)->setPen(myPen);
    ui->plot_pfoot->graph(5)->setPen(dotPen);

    ui->plot_pfoot->xAxis->setTicker(timeTicker);
    ui->plot_pfoot->axisRect()->setupFullAxesBox();
    ui->plot_pfoot->yAxis->setRange(-0.5, 0.5);
    connect(ui->plot_pfoot, SIGNAL(selectionChangedByUser()), this, SLOT(selectionChanged(ui->plot_pfoot)));
    connect(ui->plot_pfoot, SIGNAL(mousePress(QMouseEvent*)), this, SLOT(mousePress(ui->plot_pfoot)));
    connect(ui->plot_pfoot, SIGNAL(mouseWheel(QWheelEvent*)), this, SLOT(mouseWheel(ui->plot_pfoot)));
    connect(ui->plot_pfoot->xAxis, SIGNAL(rangeChanged(QCPRange)), ui->plot_pfoot->xAxis2, SLOT(setRange(QCPRange)));
    connect(ui->plot_pfoot->yAxis, SIGNAL(rangeChanged(QCPRange)), ui->plot_pfoot->yAxis2, SLOT(setRange(QCPRange)));

    // com position
    for(int i=0; i<6; i++)
       ui->plot_pcom->addGraph();
    myPen.setColor(Qt::red);
    dotPen.setColor(Qt::red);
    ui->plot_pcom->graph(0)->setPen(myPen);
    ui->plot_pcom->graph(3)->setPen(dotPen);
    myPen.setColor(Qt::blue);
    dotPen.setColor(Qt::blue);
    ui->plot_pcom->graph(1)->setPen(myPen);
    ui->plot_pcom->graph(4)->setPen(dotPen);
    myPen.setColor(Qt::green);
    dotPen.setColor(Qt::green);
    ui->plot_pcom->graph(2)->setPen(myPen);
    ui->plot_pcom->graph(5)->setPen(dotPen);

    ui->plot_pcom->xAxis->setTicker(timeTicker);
    ui->plot_pcom->axisRect()->setupFullAxesBox();
    ui->plot_pcom->yAxis->setRange(-0.5, 0.5);
    connect(ui->plot_pcom, SIGNAL(selectionChangedByUser()), this, SLOT(selectionChanged(ui->plot_pcom)));
    connect(ui->plot_pcom, SIGNAL(mousePress(QMouseEvent*)), this, SLOT(mousePress(ui->plot_pcom)));
    connect(ui->plot_pcom, SIGNAL(mouseWheel(QWheelEvent*)), this, SLOT(mouseWheel(ui->plot_pcom)));
    connect(ui->plot_pcom->xAxis, SIGNAL(rangeChanged(QCPRange)), ui->plot_pcom->xAxis2, SLOT(setRange(QCPRange)));
    connect(ui->plot_pcom->yAxis, SIGNAL(rangeChanged(QCPRange)), ui->plot_pcom->yAxis2, SLOT(setRange(QCPRange)));

    // joint angle
    for(int i=0; i<9; i++)
       ui->plot_joint->addGraph();
    myPen.setColor(Qt::red);
    dotPen.setColor(Qt::red);
    filterPen.setColor(Qt::red);
    ui->plot_joint->graph(0)->setPen(myPen);
    ui->plot_joint->graph(3)->setPen(dotPen);
    ui->plot_joint->graph(6)->setPen(filterPen);
    myPen.setColor(Qt::blue);
    dotPen.setColor(Qt::blue);
    filterPen.setColor(Qt::blue);
    ui->plot_joint->graph(1)->setPen(myPen);
    ui->plot_joint->graph(4)->setPen(dotPen);
    ui->plot_joint->graph(7)->setPen(filterPen);
    myPen.setColor(Qt::green);
    dotPen.setColor(Qt::green);
    filterPen.setColor(Qt::green);
    ui->plot_joint->graph(2)->setPen(myPen);
    ui->plot_joint->graph(5)->setPen(dotPen);
    ui->plot_joint->graph(8)->setPen(filterPen);

    ui->plot_joint->xAxis->setTicker(timeTicker);
    ui->plot_joint->axisRect()->setupFullAxesBox();
    ui->plot_joint->yAxis->setRange(-90, 90);
    connect(ui->plot_joint, SIGNAL(selectionChangedByUser()), this, SLOT(selectionChanged(ui->plot_joint)));
    connect(ui->plot_joint, SIGNAL(mousePress(QMouseEvent*)), this, SLOT(mousePress(ui->plot_joint)));
    connect(ui->plot_joint, SIGNAL(mouseWheel(QWheelEvent*)), this, SLOT(mouseWheel(ui->plot_joint)));
    connect(ui->plot_joint->xAxis, SIGNAL(rangeChanged(QCPRange)), ui->plot_joint->xAxis2, SLOT(setRange(QCPRange)));
    connect(ui->plot_joint->yAxis, SIGNAL(rangeChanged(QCPRange)), ui->plot_joint->yAxis2, SLOT(setRange(QCPRange)));

    // imu sensor
    for(int i=0; i<6; i++)
       ui->plot_imu->addGraph();
    myPen.setColor(Qt::red);
    ui->plot_imu->graph(0)->setPen(myPen);
    myPen.setColor(Qt::darkRed);
    ui->plot_imu->graph(3)->setPen(myPen);
    myPen.setColor(Qt::blue);
    ui->plot_imu->graph(1)->setPen(myPen);
    myPen.setColor(Qt::darkBlue);
    ui->plot_imu->graph(4)->setPen(myPen);
    myPen.setColor(Qt::green);
    ui->plot_imu->graph(2)->setPen(myPen);
    myPen.setColor(Qt::darkGreen);
    ui->plot_imu->graph(5)->setPen(myPen);

    ui->plot_imu->xAxis->setTicker(timeTicker);
    ui->plot_imu->axisRect()->setupFullAxesBox();
    ui->plot_imu->yAxis->setRange(-90, 90);
    connect(ui->plot_imu, SIGNAL(selectionChangedByUser()), this, SLOT(selectionChanged(ui->plot_imu)));
    connect(ui->plot_imu, SIGNAL(mousePress(QMouseEvent*)), this, SLOT(mousePress(ui->plot_imu)));
    connect(ui->plot_imu, SIGNAL(mouseWheel(QWheelEvent*)), this, SLOT(mouseWheel(ui->plot_imu)));
    connect(ui->plot_imu->xAxis, SIGNAL(rangeChanged(QCPRange)), ui->plot_imu->xAxis2, SLOT(setRange(QCPRange)));
    connect(ui->plot_imu->yAxis, SIGNAL(rangeChanged(QCPRange)), ui->plot_imu->yAxis2, SLOT(setRange(QCPRange)));

    // com control
    for(int i=0; i<6; i++)
       ui->plot_comcon->addGraph();
    myPen.setColor(Qt::red);        // angle
    ui->plot_comcon->graph(0)->setPen(myPen);
    myPen.setColor(Qt::blue);    // angle vel
    ui->plot_comcon->graph(1)->setPen(myPen);
    myPen.setColor(Qt::darkRed);      // angle vel out
    ui->plot_comcon->graph(2)->setPen(myPen);
    myPen.setColor(Qt::darkBlue);  // angle pos
    ui->plot_comcon->graph(3)->setPen(myPen);
    myPen.setColor(Qt::magenta);       // angle pos hpf
    ui->plot_comcon->graph(4)->setPen(myPen);
    myPen.setColor(Qt::green);
    ui->plot_comcon->graph(5)->setPen(myPen);

    ui->plot_comcon->xAxis->setTicker(timeTicker);
    ui->plot_comcon->axisRect()->setupFullAxesBox();
    ui->plot_comcon->yAxis->setRange(-1, 1);
    connect(ui->plot_comcon, SIGNAL(selectionChangedByUser()), this, SLOT(selectionChanged(ui->plot_comcon)));
    connect(ui->plot_comcon, SIGNAL(mousePress(QMouseEvent*)), this, SLOT(mousePress(ui->plot_comcon)));
    connect(ui->plot_comcon, SIGNAL(mouseWheel(QWheelEvent*)), this, SLOT(mouseWheel(ui->plot_comcon)));
    connect(ui->plot_comcon->xAxis, SIGNAL(rangeChanged(QCPRange)), ui->plot_comcon->xAxis2, SLOT(setRange(QCPRange)));
    connect(ui->plot_comcon->yAxis, SIGNAL(rangeChanged(QCPRange)), ui->plot_comcon->yAxis2, SLOT(setRange(QCPRange)));


    // Foot contact
    for(int i=0; i<8; i++)
       ui->plot_fcontact->addGraph();
    myPen.setColor(Qt::red);
    ui->plot_fcontact->graph(0)->setPen(myPen);
    myPen.setColor(Qt::darkRed);
    ui->plot_fcontact->graph(1)->setPen(myPen);
    myPen.setColor(Qt::blue);
    ui->plot_fcontact->graph(2)->setPen(myPen);
    myPen.setColor(Qt::darkBlue);
    ui->plot_fcontact->graph(3)->setPen(myPen);
    myPen.setColor(Qt::green);
    ui->plot_fcontact->graph(4)->setPen(myPen);
    myPen.setColor(Qt::darkGreen);
    ui->plot_fcontact->graph(5)->setPen(myPen);
    myPen.setColor(Qt::magenta);
    ui->plot_fcontact->graph(6)->setPen(myPen);
    myPen.setColor(Qt::darkMagenta);
    ui->plot_fcontact->graph(7)->setPen(myPen);

    ui->plot_fcontact->xAxis->setTicker(timeTicker);
    ui->plot_fcontact->axisRect()->setupFullAxesBox();
    ui->plot_fcontact->yAxis->setRange(-200, 200);
    connect(ui->plot_fcontact, SIGNAL(selectionChangedByUser()), this, SLOT(selectionChanged(ui->plot_fcontact)));
    connect(ui->plot_fcontact, SIGNAL(mousePress(QMouseEvent*)), this, SLOT(mousePress(ui->plot_fcontact)));
    connect(ui->plot_fcontact, SIGNAL(mouseWheel(QWheelEvent*)), this, SLOT(mouseWheel(ui->plot_fcontact)));
    connect(ui->plot_fcontact->xAxis, SIGNAL(rangeChanged(QCPRange)), ui->plot_fcontact->xAxis2, SLOT(setRange(QCPRange)));
    connect(ui->plot_fcontact->yAxis, SIGNAL(rangeChanged(QCPRange)), ui->plot_fcontact->yAxis2, SLOT(setRange(QCPRange)));


    // MPC part
    ui->plot_pb->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP :: iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->plot_dpb->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP :: iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->plot_angle->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP :: iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->plot_omega->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP :: iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->plot_contact->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP :: iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->plot_MPCfrq->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP :: iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->plot_LFfitting->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP :: iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->plot_LFextF->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP :: iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);
    ui->plot_GRF->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP :: iSelectAxes | QCP::iSelectLegend | QCP::iSelectPlottables);

    // pb
    for(int i=0; i<6; i++)
       ui->plot_pb->addGraph();
    myPen.setColor(Qt::red);
    ui->plot_pb->graph(0)->setPen(myPen);
    myPen.setColor(Qt::green);
    ui->plot_pb->graph(1)->setPen(myPen);
    myPen.setColor(Qt::blue);
    ui->plot_pb->graph(2)->setPen(myPen);
    ui->plot_pb->graph(3)->setPen(dotPen);
    ui->plot_pb->graph(4)->setPen(dotPen);
    ui->plot_pb->graph(5)->setPen(dotPen);
    ui->plot_pb->xAxis->setTicker(timeTicker);
    ui->plot_pb->axisRect()->setupFullAxesBox();
    ui->plot_pb->yAxis->setRange(-0.5, 0.5);
    connect(ui->plot_pb, SIGNAL(selectionChangedByUser()), this, SLOT(selectionChanged(ui->plot_pb)));
    connect(ui->plot_pb, SIGNAL(mousePress(QMouseEvent*)), this, SLOT(mousePress(ui->plot_pb)));
    connect(ui->plot_pb, SIGNAL(mouseWheel(QWheelEvent*)), this, SLOT(mouseWheel(ui->plot_pb)));
    connect(ui->plot_pb->xAxis, SIGNAL(rangeChanged(QCPRange)), ui->plot_pb->xAxis2, SLOT(setRange(QCPRange)));
    connect(ui->plot_pb->yAxis, SIGNAL(rangeChanged(QCPRange)), ui->plot_pb->yAxis2, SLOT(setRange(QCPRange)));


    // dpb
    for(int i=0; i<6; i++)
       ui->plot_dpb->addGraph();
    myPen.setColor(Qt::red);
    ui->plot_dpb->graph(0)->setPen(myPen);
    myPen.setColor(Qt::green);
    ui->plot_dpb->graph(1)->setPen(myPen);
    myPen.setColor(Qt::blue);
    ui->plot_dpb->graph(2)->setPen(myPen);
    ui->plot_dpb->graph(3)->setPen(dotPen);
    ui->plot_dpb->graph(4)->setPen(dotPen);
    ui->plot_dpb->graph(5)->setPen(dotPen);
    ui->plot_dpb->xAxis->setTicker(timeTicker);
    ui->plot_dpb->axisRect()->setupFullAxesBox();
    ui->plot_dpb->yAxis->setRange(-1, 1);
    connect(ui->plot_dpb, SIGNAL(selectionChangedByUser()), this, SLOT(selectionChanged(ui->plot_dpb)));
    connect(ui->plot_dpb, SIGNAL(mousePress(QMouseEvent*)), this, SLOT(mousePress(ui->plot_dpb)));
    connect(ui->plot_dpb, SIGNAL(mouseWheel(QWheelEvent*)), this, SLOT(mouseWheel(ui->plot_dpb)));
    connect(ui->plot_dpb->xAxis, SIGNAL(rangeChanged(QCPRange)), ui->plot_dpb->xAxis2, SLOT(setRange(QCPRange)));
    connect(ui->plot_dpb->yAxis, SIGNAL(rangeChanged(QCPRange)), ui->plot_dpb->yAxis2, SLOT(setRange(QCPRange)));


    // angle
    for(int i=0; i<6; i++)
       ui->plot_angle->addGraph();
    myPen.setColor(Qt::red);
    ui->plot_angle->graph(0)->setPen(myPen);
    myPen.setColor(Qt::green);
    ui->plot_angle->graph(1)->setPen(myPen);
    myPen.setColor(Qt::blue);
    ui->plot_angle->graph(2)->setPen(myPen);
    ui->plot_angle->graph(3)->setPen(dotPen);
    ui->plot_angle->graph(4)->setPen(dotPen);
    ui->plot_angle->graph(5)->setPen(dotPen);
    ui->plot_angle->xAxis->setTicker(timeTicker);
    ui->plot_angle->axisRect()->setupFullAxesBox();
    ui->plot_angle->yAxis->setRange(-90, 90);
    connect(ui->plot_angle, SIGNAL(selectionChangedByUser()), this, SLOT(selectionChanged(ui->plot_angle)));
    connect(ui->plot_angle, SIGNAL(mousePress(QMouseEvent*)), this, SLOT(mousePress(ui->plot_angle)));
    connect(ui->plot_angle, SIGNAL(mouseWheel(QWheelEvent*)), this, SLOT(mouseWheel(ui->plot_angle)));
    connect(ui->plot_angle->xAxis, SIGNAL(rangeChanged(QCPRange)), ui->plot_angle->xAxis2, SLOT(setRange(QCPRange)));
    connect(ui->plot_angle->yAxis, SIGNAL(rangeChanged(QCPRange)), ui->plot_angle->yAxis2, SLOT(setRange(QCPRange)));


    // omega
    for(int i=0; i<6; i++)
       ui->plot_omega->addGraph();
    myPen.setColor(Qt::red);
    ui->plot_omega->graph(0)->setPen(myPen);
    myPen.setColor(Qt::green);
    ui->plot_omega->graph(1)->setPen(myPen);
    myPen.setColor(Qt::blue);
    ui->plot_omega->graph(2)->setPen(myPen);
    ui->plot_omega->graph(3)->setPen(dotPen);
    ui->plot_omega->graph(4)->setPen(dotPen);
    ui->plot_omega->graph(5)->setPen(dotPen);
    ui->plot_omega->xAxis->setTicker(timeTicker);
    ui->plot_omega->axisRect()->setupFullAxesBox();
    ui->plot_omega->yAxis->setRange(-0.5, 0.5);
    connect(ui->plot_omega, SIGNAL(selectionChangedByUser()), this, SLOT(selectionChanged(ui->plot_omega)));
    connect(ui->plot_omega, SIGNAL(mousePress(QMouseEvent*)), this, SLOT(mousePress(ui->plot_omega)));
    connect(ui->plot_omega, SIGNAL(mouseWheel(QWheelEvent*)), this, SLOT(mouseWheel(ui->plot_omega)));
    connect(ui->plot_omega->xAxis, SIGNAL(rangeChanged(QCPRange)), ui->plot_omega->xAxis2, SLOT(setRange(QCPRange)));
    connect(ui->plot_omega->yAxis, SIGNAL(rangeChanged(QCPRange)), ui->plot_omega->yAxis2, SLOT(setRange(QCPRange)));


    //contact
    for(int i=0; i<6; i++)
       ui->plot_contact->addGraph();
    myPen.setColor(Qt::red);
    ui->plot_contact->graph(0)->setPen(myPen);
    ui->plot_contact->graph(0)->setLineStyle(QCPGraph::lsStepLeft);
    ui->plot_contact->graph(0)->setBrush(QColor(50,50,50,200));
    myPen.setColor(Qt::red);
    ui->plot_contact->graph(1)->setPen(myPen);
    ui->plot_contact->graph(1)->setLineStyle(QCPGraph::lsStepLeft);
    ui->plot_contact->graph(1)->setBrush(QColor(100,100,100,150));
    myPen.setColor(Qt::red);
    ui->plot_contact->graph(2)->setPen(myPen);
    ui->plot_contact->graph(2)->setLineStyle(QCPGraph::lsStepLeft);
    ui->plot_contact->graph(2)->setBrush(QColor(150,150,150,100));
    myPen.setColor(Qt::red);
    ui->plot_contact->graph(3)->setPen(myPen);
    ui->plot_contact->graph(3)->setLineStyle(QCPGraph::lsStepLeft);
    ui->plot_contact->graph(3)->setBrush(QColor(200,200,200,50));
    myPen.setColor(QColor(255,255,255,0));
    ui->plot_contact->graph(4)->setPen(myPen);
    ui->plot_contact->graph(4)->setLineStyle(QCPGraph::lsStepLeft);
    ui->plot_contact->graph(4)->setBrush(QColor(255,255,255,30));
    myPen.setColor(QColor(255,255,255,0));
    ui->plot_contact->graph(5)->setPen(myPen);
    ui->plot_contact->graph(5)->setLineStyle(QCPGraph::lsStepLeft);
    ui->plot_contact->graph(5)->setBrush(QColor(100,200,250,50));
    ui->plot_contact->xAxis->setTicker(timeTicker);
    ui->plot_contact->axisRect()->setupFullAxesBox();
    ui->plot_contact->yAxis->setRange(0, 5);
    connect(ui->plot_contact, SIGNAL(selectionChangedByUser()), this, SLOT(selectionChanged(ui->plot_contact)));
    connect(ui->plot_contact, SIGNAL(mousePress(QMouseEvent*)), this, SLOT(mousePress(ui->plot_contact)));
    connect(ui->plot_contact, SIGNAL(mouseWheel(QWheelEvent*)), this, SLOT(mouseWheel(ui->plot_contact)));
    connect(ui->plot_contact->xAxis, SIGNAL(rangeChanged(QCPRange)), ui->plot_contact->xAxis2, SLOT(setRange(QCPRange)));
    connect(ui->plot_contact->yAxis, SIGNAL(rangeChanged(QCPRange)), ui->plot_contact->yAxis2, SLOT(setRange(QCPRange)));


//    // MPC frequency
//    myPen.setColor(Qt::red);
//    ui->plot_MPCfrq->addGraph(); // red line
//    ui->plot_MPCfrq->graph(0)->setPen(myPen);
//    ui->plot_MPCfrq->xAxis->setTicker(timeTicker);
//    ui->plot_MPCfrq->axisRect()->setupFullAxesBox();
//    ui->plot_MPCfrq->yAxis->setRange(-10, 300);
//    connect(ui->plot_MPCfrq, SIGNAL(selectionChangedByUser()), this, SLOT(selectionChanged(ui->plot_MPCfrq)));
//    connect(ui->plot_MPCfrq, SIGNAL(mousePress(QMouseEvent*)), this, SLOT(mousePress(ui->plot_MPCfrq)));
//    connect(ui->plot_MPCfrq, SIGNAL(mouseWheel(QWheelEvent*)), this, SLOT(mouseWheel(ui->plot_MPCfrq)));
//    connect(ui->plot_MPCfrq->xAxis, SIGNAL(rangeChanged(QCPRange)), ui->plot_MPCfrq->xAxis2, SLOT(setRange(QCPRange)));
//    connect(ui->plot_MPCfrq->yAxis, SIGNAL(rangeChanged(QCPRange)), ui->plot_MPCfrq->yAxis2, SLOT(setRange(QCPRange)));


//    // LF acc2trq fitting
//    for(int i=0; i<3; i++)
//       ui->plot_LFfitting->addGraph();
//    myPen.setColor(Qt::red);
//    ui->plot_LFfitting->graph(0)->setPen(myPen);
//    myPen.setColor(Qt::green);
//    ui->plot_LFfitting->graph(1)->setPen(myPen);
//    myPen.setColor(Qt::blue);
//    ui->plot_LFfitting->graph(2)->setPen(myPen);
//    ui->plot_LFfitting->xAxis->setTicker(timeTicker);
//    ui->plot_LFfitting->axisRect()->setupFullAxesBox();
//    ui->plot_LFfitting->yAxis->setRange(-10.0, 10.0);
//    connect(ui->plot_LFfitting, SIGNAL(selectionChangedByUser()), this, SLOT(selectionChanged(ui->plot_LFfitting)));
//    connect(ui->plot_LFfitting, SIGNAL(mousePress(QMouseEvent*)), this, SLOT(mousePress(ui->plot_LFfitting)));
//    connect(ui->plot_LFfitting, SIGNAL(mouseWheel(QWheelEvent*)), this, SLOT(mouseWheel(ui->plot_LFfitting)));
//    connect(ui->plot_LFfitting->xAxis, SIGNAL(rangeChanged(QCPRange)), ui->plot_LFfitting->xAxis2, SLOT(setRange(QCPRange)));
//    connect(ui->plot_LFfitting->yAxis, SIGNAL(rangeChanged(QCPRange)), ui->plot_LFfitting->yAxis2, SLOT(setRange(QCPRange)));


//    // LF external force
//    for(int i=0; i<5; i++)
//       ui->plot_LFextF->addGraph();
//    myPen.setColor(Qt::red);
//    ui->plot_LFextF->graph(0)->setPen(myPen);
//    myPen.setColor(Qt::green);
//    ui->plot_LFextF->graph(1)->setPen(myPen);
//    myPen.setColor(Qt::blue);
//    ui->plot_LFextF->graph(2)->setPen(myPen);
//    myPen.setColor(QColor(255,255,255,0));
//    ui->plot_LFextF->graph(3)->setPen(myPen);
//    ui->plot_LFextF->graph(3)->setLineStyle(QCPGraph::lsStepLeft);
//    ui->plot_LFextF->graph(3)->setBrush(QColor(255,255,255,30));
//    myPen.setColor(QColor(255,255,255,0));
//    ui->plot_LFextF->graph(4)->setPen(myPen);
//    ui->plot_LFextF->graph(4)->setLineStyle(QCPGraph::lsStepLeft);
//    ui->plot_LFextF->graph(4)->setBrush(QColor(100,200,250,50));
//    ui->plot_LFextF->xAxis->setTicker(timeTicker);
//    ui->plot_LFextF->axisRect()->setupFullAxesBox();
//    ui->plot_LFextF->yAxis->setRange(-200, 200);
//    connect(ui->plot_LFextF, SIGNAL(selectionChangedByUser()), this, SLOT(selectionChanged(ui->plot_LFextF)));
//    connect(ui->plot_LFextF, SIGNAL(mousePress(QMouseEvent*)), this, SLOT(mousePress(ui->plot_LFextF)));
//    connect(ui->plot_LFextF, SIGNAL(mouseWheel(QWheelEvent*)), this, SLOT(mouseWheel(ui->plot_LFextF)));
//    connect(ui->plot_LFextF->xAxis, SIGNAL(rangeChanged(QCPRange)), ui->plot_LFextF->xAxis2, SLOT(setRange(QCPRange)));
//    connect(ui->plot_LFextF->yAxis, SIGNAL(rangeChanged(QCPRange)), ui->plot_LFextF->yAxis2, SLOT(setRange(QCPRange)));


//    // LF GRF from MPC
//    for(int i=0; i<5; i++)
//       ui->plot_GRF->addGraph();
//    myPen.setColor(Qt::red);
//    ui->plot_GRF->graph(0)->setPen(myPen);
//    myPen.setColor(Qt::green);
//    ui->plot_GRF->graph(1)->setPen(myPen);
//    myPen.setColor(Qt::blue);
//    ui->plot_GRF->graph(2)->setPen(myPen);
//    myPen.setColor(QColor(255,255,255,0));
//    ui->plot_GRF->graph(3)->setPen(myPen);
//    ui->plot_GRF->graph(3)->setLineStyle(QCPGraph::lsStepLeft);
//    ui->plot_GRF->graph(3)->setBrush(QColor(255,255,255,30));
//    myPen.setColor(QColor(255,255,255,0));
//    ui->plot_GRF->graph(4)->setPen(myPen);
//    ui->plot_GRF->graph(4)->setLineStyle(QCPGraph::lsStepLeft);
//    ui->plot_GRF->graph(4)->setBrush(QColor(100,200,250,50));
//    ui->plot_GRF->xAxis->setTicker(timeTicker);
//    ui->plot_GRF->axisRect()->setupFullAxesBox();
//    ui->plot_GRF->yAxis->setRange(-200, 200);
//    connect(ui->plot_GRF, SIGNAL(selectionChangedByUser()), this, SLOT(selectionChanged(ui->plot_GRF)));
//    connect(ui->plot_GRF, SIGNAL(mousePress(QMouseEvent*)), this, SLOT(mousePress(ui->plot_GRF)));
//    connect(ui->plot_GRF, SIGNAL(mouseWheel(QWheelEvent*)), this, SLOT(mouseWheel(ui->plot_GRF)));
//    connect(ui->plot_GRF->xAxis, SIGNAL(rangeChanged(QCPRange)), ui->plot_GRF->xAxis2, SLOT(setRange(QCPRange)));
//    connect(ui->plot_GRF->yAxis, SIGNAL(rangeChanged(QCPRange)), ui->plot_GRF->yAxis2, SLOT(setRange(QCPRange)));

}

void MainWindow::DispalyUpdate_BasicTest(){
    static QTime time(QTime::currentTime());
    QString str;

    // Joystick part
    cnt_joy++;
    //if(cnt_joy%5==0)
    if(true)
    {
        double moveX, moveY, moveZup, moveZdown, rotatePitch, rotateYaw;
        if(joy->connection)
        {
            moveX = calc_joy(-joy->JoyAxis[1])*ui->LE_JOYX->text().toDouble();
            moveY = calc_joy(-joy->JoyAxis[0])*ui->LE_JOYY->text().toDouble();
            moveZup = (calc_joy(joy->JoyAxis[2])+1)*ui->LE_JOYZ->text().toDouble()/2;
            moveZdown = (calc_joy(joy->JoyAxis[5])+1)*ui->LE_JOYZ->text().toDouble()/2;
//            moveZdown = (joy->JoyAxis[5])*ui->LE_JOYXY->text().toDouble();
            rotatePitch = calc_joy(-joy->JoyAxis[4])*ui->LE_JOYANGLE->text().toDouble();
            rotateYaw = calc_joy(-joy->JoyAxis[3])*ui->LE_JOYANGLE->text().toDouble();

            if(ui->CB_Jcon->isChecked())
            {
                sharedData->joy_ON = true;

                sharedData->joy_dpCOM[0] = moveX;
                sharedData->joy_dpCOM[1] = moveY;
                sharedData->joy_dpCOM[2] = rotatePitch;
                sharedData->joy_dpCOM[3] = rotateYaw;

                sharedData->joy_dRPY[0] = calc_joy(-joy->JoyAxis[0]);
                sharedData->joy_dRPY[1] = calc_joy(-joy->JoyAxis[1]);
                sharedData->joy_dRPY[2] = calc_joy(-joy->JoyAxis[3]);

                if(joy->JoyAxis[6] > 0) sharedData->joy_right = 1;
                else if(joy->JoyAxis[6] < 0) sharedData->joy_left = 1;
                else {sharedData->joy_left = 0; sharedData->joy_right = 0;}

                if(joy->JoyAxis[7] > 0) sharedData->joy_down = 1;
                else if(joy->JoyAxis[7] < 0) sharedData->joy_up = 1;
                else {sharedData->joy_down = 0; sharedData->joy_up = 0;}

                if(sharedData->joy_up == true) {
                    sharedData->Walking_period = 0.15;
                }
                if(sharedData->joy_down == true) {
                    sharedData->Walking_period = 0.3;
                }
                if(sharedData->joy_left == true) {
                    sharedData->Walking_period = 0.2;
                }
                if(sharedData->joy_right == true) {
                    sharedData->Walking_period = 0.25;
                }

//                cout << "period : " << sharedData->Walking_period << endl;

//                cout << "l:" << sharedData->joy_left << ", r:" << sharedData->joy_right << ", d:" << sharedData->joy_down << ", u:" << sharedData->joy_up << endl;

                // A mapping
                if((joy->JoyButton[J_A] == true && joy->JoyButton_pre[J_A] == false)) {
                    joy->State = J_WALKSTART;
                }
                if(joy->JoyButton[J_LB] == true && (joy->JoyButton[J_A] == true && joy->JoyButton_pre[J_A] == false)) {
                    joy->State = J_GROUNDPOS;
                }
                if(joy->JoyButton[J_RB] == true && (joy->JoyButton[J_A] == true && joy->JoyButton_pre[J_A] == false)) {
                    joy->State = J_BACKFLIP;
                }

                // B mapping
                if((joy->JoyButton[J_B] == true && joy->JoyButton_pre[J_B] == false)) {
                    joy->State = J_WALKSTOP;
                }
                if(joy->JoyButton[J_LB] == true && (joy->JoyButton[J_B] == true && joy->JoyButton_pre[J_B] == false)) {
                    joy->State = J_MOTIONSTOP;
                }

                // X mapping
                if((joy->JoyButton[J_X] == true && joy->JoyButton_pre[J_X] == false)) {
                    joy->State = J_WALKREADY;
                }
                if(joy->JoyButton[J_LB] == true && (joy->JoyButton[J_X] == true && joy->JoyButton_pre[J_X] == false)) {
                    joy->State = J_READYPOS;
                }

                // Y mapping
                if((joy->JoyButton[J_Y] == true && joy->JoyButton_pre[J_Y] == false)) {
                    joy->State = J_IMURESET;
                }
                if(joy->JoyButton[J_LB] == true && (joy->JoyButton[J_Y] == true && joy->JoyButton_pre[J_Y] == false)) {
                    joy->State = J_IMUNULLING;
                }
                if(joy->JoyButton[J_RB] == true && (joy->JoyButton[J_Y] == true && joy->JoyButton_pre[J_Y] == false)) {
                    joy->State = J_FALLRECOVER;
                }

                // Start, Back mapping
                if((joy->JoyButton[J_START] == true && joy->JoyButton_pre[J_START] == false)) {
                    joy->State = J_TROTWALKING;
                }
                if(joy->JoyButton[J_LB] == true && (joy->JoyButton[J_START] == true && joy->JoyButton_pre[J_START] == false)) {
                    joy->State = J_TROTWALKING_S;
                }
                if((joy->JoyButton[J_BACK] == true && joy->JoyButton_pre[J_BACK] == false)) {
                    joy->State = J_F_TROTWALKING;
                }
                // Logi mapping
                if((joy->JoyButton[J_LOGI] == true && joy->JoyButton_pre[J_LOGI] == false)) {
                    joy->State = J_PRONKING;
                }

                // Task mapping

                if(joy->State == J_READYPOS) {
                    joy->State = J_NOACT;
                    cout << "READYPOS" << endl;
                    on_BTN_READYPOS_clicked();
                }
                else if(joy->State == J_GROUNDPOS) {
                    joy->State = J_NOACT;
                    cout << "GROUNDPOS" << endl;
                    on_BTN_GROUNDPOS_clicked();
                }
                else if(joy->State == J_MOTIONSTOP) {
                    joy->State = J_NOACT;
                    cout << "MOTIONSTOP" << endl;
                    on_BTN_STOP_clicked();
                }
                else if(joy->State == J_BACKFLIP) {
                    joy->State = J_NOACT;
                    cout << "BACKFLIP" << endl;
                    on_BTN_BACKFLIP_clicked();
                }
                else if(joy->State == J_IMUNULLING) {
                    joy->State = J_NOACT;
                    if(sharedData->trot_variable_int[0] == 1) {
                        FILE_LOG(logWARNING) << "is Walking!";
                    }
                    else {
                        cout << "IMUNULLING" << endl;
                        on_BTN_IMU_NULL_clicked();
                    }
                }
                else if(joy->State == J_FALLRECOVER) {
                    joy->State = J_NOACT;
                    if(sharedData->isFalled == false) {
                        FILE_LOG(logWARNING) << "not Falled!";
                    }
                    else {
                        cout << "FALLRECOVER" << endl;
                        on_BTN_FALLRECOVER_clicked();
                    }
                }
                else if(joy->State == J_WALKREADY) {
                    joy->State = J_NOACT;
                    cout << "WALKREADY" << endl;
                    if(sharedData->trot_variable_int[2] == 3) {
                        on_BTN_HOPTEST_READY_clicked();
                    }
                    else {
                        on_BTN_TROTTEST_READY_clicked();
                    }

                }
                else if(joy->State == J_WALKSTART) {
                    joy->State = J_NOACT;
                    cout << "WALKSTART" << endl;
                    if(sharedData->trot_variable_int[2] == 3) {
                        on_BTN_HOPTEST_START_clicked();
                    }
                    else {
                        on_BTN_TROTTEST_START_clicked();
                    }
                }
                else if(joy->State == J_WALKSTOP) {
                    joy->State = J_NOACT;
                    cout << "WALKSTOP" << endl;
                    if(sharedData->trot_variable_int[2] == 3) {
                        on_BTN_HOPTEST_STOP_clicked();
                    }
                    else {
                        on_BTN_TROTTEST_STOP_clicked();
                    }
                }
                else if(joy->State == J_IMURESET) {
                    joy->State = J_NOACT;
                    if(sharedData->trot_variable_int[0] == 1) {
                        FILE_LOG(logWARNING) << "is Walking!";
                    }
                    else {
                        cout << "IMURESET" << endl;
                        on_BTN_IMU_RESET_clicked();
                    }
                }
                else if(joy->State == J_TROTWALKING) {
                    joy->State = J_NOACT;
                    cout << "TROTWALKING" << endl;
                    sharedData->trot_variable_int[2] = 0;
                }
                else if(joy->State == J_TROTWALKING_S) {
                    joy->State = J_NOACT;
                    cout << "TROTWALKING_S" << endl;
                    sharedData->trot_variable_int[2] = 2;
                }
                else if(joy->State == J_F_TROTWALKING) {
                    joy->State = J_NOACT;
                    cout << "F_TROTWALKING" << endl;
                    sharedData->trot_variable_int[2] = 1;
                }
                else if(joy->State == J_PRONKING) {
                    joy->State = J_NOACT;
                    cout << "PRONKING" << endl;
                    sharedData->trot_variable_int[2] = 3;
                }

                for(int i=0; i<12; i++) {
                    joy->JoyButton_pre[i] = joy->JoyButton[i];
                }
            }
            else {
                sharedData->joy_ON = false;
                sharedData->joy_dpCOM[0] = 0;
                sharedData->joy_dpCOM[1] = 0;
                sharedData->joy_dpCOM[2] = 0;
                sharedData->joy_dRPY[0] = 0;
                sharedData->joy_dRPY[1] = 0;
                sharedData->joy_dRPY[2] = 0;
            }

        }
        else {
            moveX = 0;
            moveY = 0;
            moveZup = 0;
            moveZdown = 0;
            rotatePitch = 0;
            rotateYaw = 0;
        }

        QString str2,str3,str4;
        if(joy->connection) { str4 = "connected\n";   }
        else{ str4 = "not connected\n";  }
        str2.sprintf("BUTTONS %d%d%d%d%d%d %d%d%d%d%d%d\n"
                ,(int)joy->JoyButton[0],(int)joy->JoyButton[1],(int)joy->JoyButton[2]
                ,(int)joy->JoyButton[3],(int)joy->JoyButton[4],(int)joy->JoyButton[5]
                ,(int)joy->JoyButton[6],(int)joy->JoyButton[7],(int)joy->JoyButton[8]
                ,(int)joy->JoyButton[9],(int)joy->JoyButton[10],(int)joy->JoyButton[11]);
        str3.sprintf("AXIS\n dx: %.3f dz+: %.3f dP: %.3f \ndy: %.3f dz-: %.3f dY %.3f"
                ,moveX,moveZup,rotatePitch,moveY, moveZdown, rotateYaw);
        str2 = str4+str2+str3;
        ui->TL_JOYVIEW->setText(str2);

        cnt_joy = 0;
    }

    double key = time.elapsed()/1000.0;
    static double lastPointKey = 0;

    int idx_pf = ui->Combo_pfoot->currentIndex();
    int idx_pf_mode = ui->Combo_pfoot_mode->currentIndex();

    int idx_pcom = ui->Combo_pcom->currentIndex();
    int idx_pcom_mode = ui->Combo_pcom_mode->currentIndex();

    int idx_joint = ui->Combo_joint->currentIndex();
    int idx_joint_mode = ui->Combo_joint_mode->currentIndex();

    int idx_imu = ui->Combo_imu->currentIndex();

    int idx_IK = ui->Combo_IK_leg->currentIndex();



    if(sharedData->plot_play_ON == true)
    {
        {
            //--------------------- graph part ---------------------
            // foot position
            if(ui->CB_pfoot_x->isChecked())
            {
                ui->plot_pfoot->graph(0)->addData(key, sharedData->pf_real[idx_pf_mode*12+idx_pf*3+0]);
                if(ui->CB_plotrefON->isChecked())
                   ui->plot_pfoot->graph(3)->addData(key, sharedData->pf_ref[idx_pf_mode*12+idx_pf*3+0]);
            }
            if(ui->CB_pfoot_y->isChecked())
            {
                ui->plot_pfoot->graph(1)->addData(key, sharedData->pf_real[idx_pf_mode*12+idx_pf*3+1]);
                if(ui->CB_plotrefON->isChecked())
                   ui->plot_pfoot->graph(4)->addData(key, sharedData->pf_ref[idx_pf_mode*12+idx_pf*3+1]);
            }
            if(ui->CB_pfoot_z->isChecked())
            {
                ui->plot_pfoot->graph(2)->addData(key, sharedData->pf_real[idx_pf_mode*12+idx_pf*3+2]);
                if(ui->CB_plotrefON->isChecked())
                   ui->plot_pfoot->graph(5)->addData(key, sharedData->pf_ref[idx_pf_mode*12+idx_pf*3+2]);
            }

            // com position p 0 1 2 w 3 4 5 6 dp 7 8 9 dw 10 11
            if(ui->CB_pcom_x->isChecked())
            {
                ui->plot_pcom->graph(0)->addData(key, sharedData->pcom_real[idx_pcom_mode*6+idx_pcom*3+0]);
                if(ui->CB_plotrefON->isChecked())
                   ui->plot_pcom->graph(3)->addData(key, sharedData->pcom_ref[idx_pcom_mode*6+idx_pcom*3+0]);
            }
            if(ui->CB_pcom_y->isChecked())
            {
                ui->plot_pcom->graph(1)->addData(key, sharedData->pcom_real[idx_pcom_mode*6+idx_pcom*3+1]);
                if(ui->CB_plotrefON->isChecked())
                   ui->plot_pcom->graph(4)->addData(key, sharedData->pcom_ref[idx_pcom_mode*6+idx_pcom*3+1]);
            }
            if(ui->CB_pcom_z->isChecked())
            {
                ui->plot_pcom->graph(2)->addData(key, sharedData->pcom_real[idx_pcom_mode*6+idx_pcom*3+2]);
                if(ui->CB_plotrefON->isChecked())
                   ui->plot_pcom->graph(5)->addData(key, sharedData->pcom_ref[idx_pcom_mode*6+idx_pcom*3+2]);
            }

            double r2d;
            if(idx_joint_mode == 3) r2d = 1.0;
            else r2d = R2Df;

            // joint information
            if(ui->CB_joint_R->isChecked())
            {
                if(ui->CB_plot_raw->isChecked())
                    ui->plot_joint->graph(0)->addData(key, r2d*sharedData->joint_info[idx_joint_mode*12+idx_joint*3+0]);
                if(ui->CB_plotrefON->isChecked())
                   ui->plot_joint->graph(3)->addData(key, r2d*sharedData->joint_info_ref[idx_joint_mode*12+idx_joint*3+0]);
                if(ui->CB_plot_filtered->isChecked())
                    ui->plot_joint->graph(6)->addData(key, r2d*sharedData->joint_info_filtered[idx_joint_mode*12+idx_joint*3+0]);
            }
            if(ui->CB_joint_P->isChecked())
            {
                if(ui->CB_plot_raw->isChecked())
                    ui->plot_joint->graph(1)->addData(key, r2d*sharedData->joint_info[idx_joint_mode*12+idx_joint*3+1]);
                if(ui->CB_plotrefON->isChecked())
                   ui->plot_joint->graph(4)->addData(key, r2d*sharedData->joint_info_ref[idx_joint_mode*12+idx_joint*3+1]);
                if(ui->CB_plot_filtered->isChecked())
                    ui->plot_joint->graph(7)->addData(key, r2d*sharedData->joint_info_filtered[idx_joint_mode*12+idx_joint*3+1]);
            }
            if(ui->CB_joint_K->isChecked())
            {
                if(ui->CB_plot_raw->isChecked())
                    ui->plot_joint->graph(2)->addData(key, r2d*sharedData->joint_info[idx_joint_mode*12+idx_joint*3+2]);
                if(ui->CB_plotrefON->isChecked())
                   ui->plot_joint->graph(5)->addData(key, r2d*sharedData->joint_info_ref[idx_joint_mode*12+idx_joint*3+2]);
                if(ui->CB_plot_filtered->isChecked())
                    ui->plot_joint->graph(8)->addData(key, r2d*sharedData->joint_info_filtered[idx_joint_mode*12+idx_joint*3+2]);
            }

            // imu
            if(ui->CB_imu_roll->isChecked())
            {
                if(ui->CB_imu_filtered->isChecked()) {
                    if(idx_imu == 0)        ui->plot_imu->graph(0)->addData(key, sharedData->IMU[1].Roll);
                    else if(idx_imu == 1)   ui->plot_imu->graph(0)->addData(key, sharedData->IMU[1].RollVel);
                    else if(idx_imu == 2)   ui->plot_imu->graph(0)->addData(key, sharedData->IMU[1].AccX);
                }
                if(ui->CB_imu_raw->isChecked()) {
                    if(idx_imu == 0)        ui->plot_imu->graph(3)->addData(key, sharedData->IMU[0].Roll);
                    else if(idx_imu == 1)   ui->plot_imu->graph(3)->addData(key, sharedData->IMU[0].RollVel);
                    else if(idx_imu == 2)   ui->plot_imu->graph(3)->addData(key, sharedData->IMU[0].AccX);
                }
            }
            if(ui->CB_imu_pitch->isChecked())
            {
                if(ui->CB_imu_filtered->isChecked()) {
                    if(idx_imu == 0)        ui->plot_imu->graph(1)->addData(key, sharedData->IMU[1].Pitch);
                    else if(idx_imu == 1)   ui->plot_imu->graph(1)->addData(key, sharedData->IMU[1].PitchVel);
                    else if(idx_imu == 2)   ui->plot_imu->graph(1)->addData(key, sharedData->IMU[1].AccY);
                }
                if(ui->CB_imu_raw->isChecked()) {
                    if(idx_imu == 0)        ui->plot_imu->graph(4)->addData(key, sharedData->IMU[0].Pitch);
                    else if(idx_imu == 1)   ui->plot_imu->graph(4)->addData(key, sharedData->IMU[0].PitchVel);
                    else if(idx_imu == 2)   ui->plot_imu->graph(4)->addData(key, sharedData->IMU[0].AccY);
                }
            }
            if(ui->CB_imu_yaw->isChecked())
            {
                if(ui->CB_imu_filtered->isChecked()) {
                    if(idx_imu == 0)        ui->plot_imu->graph(2)->addData(key, sharedData->IMU[1].Yaw);
                    else if(idx_imu == 1)   ui->plot_imu->graph(2)->addData(key, sharedData->IMU[1].YawVel);
                    else if(idx_imu == 2)   ui->plot_imu->graph(2)->addData(key, sharedData->IMU[1].AccZ);
                }
                if(ui->CB_imu_raw->isChecked()) {
                    if(idx_imu == 0)        ui->plot_imu->graph(5)->addData(key, sharedData->IMU[0].Yaw);
                    else if(idx_imu == 1)   ui->plot_imu->graph(5)->addData(key, sharedData->IMU[0].YawVel);
                    else if(idx_imu == 2)   ui->plot_imu->graph(5)->addData(key, sharedData->IMU[0].AccZ);
                }
            }

            // COM Control
            if(ui->CB_comcon_1->isChecked()) ui->plot_comcon->graph(0)->addData(key, sharedData->comcon_variable_plot[0]);
            if(ui->CB_comcon_2->isChecked()) ui->plot_comcon->graph(1)->addData(key, sharedData->comcon_variable_plot[1]);
            if(ui->CB_comcon_3->isChecked()) ui->plot_comcon->graph(2)->addData(key, sharedData->comcon_variable_plot[2]);
            if(ui->CB_comcon_4->isChecked()) ui->plot_comcon->graph(3)->addData(key, sharedData->comcon_variable_plot[3]);
            if(ui->CB_comcon_5->isChecked()) ui->plot_comcon->graph(4)->addData(key, sharedData->comcon_variable_plot[4]);
            if(ui->CB_comcon_6->isChecked()) ui->plot_comcon->graph(5)->addData(key, sharedData->comcon_variable_plot[5]);

            // Foot contact
//            if(ui->CB_CONTACT_HR->isChecked()) ui->plot_fcontact->graph(0)->addData(key, sharedData->pf_real[36+2]);
//            if(ui->CB_CONTACT_HL->isChecked()) ui->plot_fcontact->graph(1)->addData(key, sharedData->pf_real[36+5]);
//            if(ui->CB_CONTACT_FR->isChecked()) ui->plot_fcontact->graph(2)->addData(key, sharedData->pf_real[36+8]);
//            if(ui->CB_CONTACT_FL->isChecked()) ui->plot_fcontact->graph(3)->addData(key, sharedData->pf_real[36+11]);
            if(ui->CB_CONTACT_HR->isChecked()) {
                ui->plot_fcontact->graph(0)->addData(key, sharedData->contact_flag[HR]);
                ui->plot_fcontact->graph(1)->addData(key, sharedData->contact_flag_test[HR]);
            }
            if(ui->CB_CONTACT_HL->isChecked()) {
                ui->plot_fcontact->graph(2)->addData(key, sharedData->contact_flag[HL]);
                ui->plot_fcontact->graph(3)->addData(key, sharedData->contact_flag_test[HL]);
            }
            if(ui->CB_CONTACT_FR->isChecked()) {
                ui->plot_fcontact->graph(4)->addData(key, sharedData->contact_flag[FR]);
                ui->plot_fcontact->graph(5)->addData(key, sharedData->contact_flag_test[FR]);
            }
            if(ui->CB_CONTACT_FL->isChecked()) {
                ui->plot_fcontact->graph(6)->addData(key, sharedData->contact_flag[FL]);
                ui->plot_fcontact->graph(7)->addData(key, sharedData->contact_flag_test[FL]);
            }

            //--------------------- text part ---------------------
            ui->LE_jpos_real_R->setText(str.sprintf("%.1f", R2Df*sharedData->joint_info[idx_IK*3+0]));
            ui->LE_jpos_real_P->setText(str.sprintf("%.1f", R2Df*sharedData->joint_info[idx_IK*3+1]));
            ui->LE_jpos_real_K->setText(str.sprintf("%.1f", R2Df*sharedData->joint_info[idx_IK*3+2]));
            ui->LE_jpos_IK_R->setText(str.sprintf("%.1f", R2Df*sharedData->joint_IK[idx_IK*3+0]));
            ui->LE_jpos_IK_P->setText(str.sprintf("%.1f", R2Df*sharedData->joint_IK[idx_IK*3+1]));
            ui->LE_jpos_IK_K->setText(str.sprintf("%.1f", R2Df*sharedData->joint_IK[idx_IK*3+2]));

            lastPointKey = key;
        }

        if(ui->CB_AutoMove->isChecked())
        {
            // graph x range - uiuk
            double offset_left = 2.0;
            double offset_right = 0.0;
            ui->plot_pfoot->xAxis->setRange(key-offset_left, key+offset_right);
            ui->plot_pcom->xAxis->setRange(key-offset_left, key+offset_right);
            ui->plot_joint->xAxis->setRange(key-offset_left, key+offset_right);
            ui->plot_imu->xAxis->setRange(key-offset_left, key+offset_right);
            ui->plot_comcon->xAxis->setRange(key-offset_left, key+offset_right);
            ui->plot_fcontact->xAxis->setRange(key-offset_left, key+offset_right);
        }

        ui->plot_pfoot->replot();
        ui->plot_pcom->replot();
        ui->plot_joint->replot();
        ui->plot_imu->replot();
        ui->plot_comcon->replot();
        ui->plot_fcontact->replot();

        if(sharedData->plot_timer_zero == true)
        {
            time.restart();
            for(int i=0; i<6; i++)
            {
                ui->plot_pfoot->graph(i)->data()->clear();
            }
            for(int i=0; i<6; i++)
            {
                ui->plot_pcom->graph(i)->data()->clear();
            }
            for(int i=0; i<9; i++)
            {
                ui->plot_joint->graph(i)->data()->clear();
            }
            for(int i=0; i<6; i++)
            {
                ui->plot_imu->graph(i)->data()->clear();
            }
            for(int i=0; i<6; i++)
            {
                ui->plot_comcon->graph(i)->data()->clear();
            }
            for(int i=0; i<8; i++)
            {
                ui->plot_fcontact->graph(i)->data()->clear();
            }

            key = 0;
            lastPointKey = 0;
            sharedData->plot_timer_zero = false;
        }
    }
}

void MainWindow::DispalyUpdate_MPC(){
    static QTime time(QTime::currentTime());
    QString str;

    double key = time.elapsed()/1000.0;
    static double lastPointKey = 0;
    static int gait_sequence_int = 1;
}

void MainWindow::selectionChanged(QCustomPlot* &ui_test)
{
    if (ui_test->xAxis->selectedParts().testFlag(QCPAxis::spAxis) || ui_test->xAxis->selectedParts().testFlag(QCPAxis::spTickLabels) ||
             ui_test->xAxis2->selectedParts().testFlag(QCPAxis::spAxis) || ui_test->xAxis2->selectedParts().testFlag(QCPAxis::spTickLabels))
     {
         ui_test->xAxis2->setSelectedParts(QCPAxis::spAxis|QCPAxis::spTickLabels);
         ui_test->xAxis->setSelectedParts(QCPAxis::spAxis|QCPAxis::spTickLabels);
     }
     if (ui_test->yAxis->selectedParts().testFlag(QCPAxis::spAxis) || ui_test->yAxis->selectedParts().testFlag(QCPAxis::spTickLabels) ||
             ui_test->yAxis2->selectedParts().testFlag(QCPAxis::spAxis)|| ui_test->yAxis2->selectedParts().testFlag(QCPAxis::spTickLabels))
     {
         ui_test->yAxis2->setSelectedParts(QCPAxis::spAxis|QCPAxis::spTickLabels);
         ui_test->yAxis->setSelectedParts(QCPAxis::spAxis|QCPAxis::spTickLabels);
     }

     for (int i=0; i<ui_test->graphCount(); ++i)
     {
         QCPGraph *graph = ui_test->graph(i);
         QCPPlottableLegendItem *item = ui_test->legend->itemWithPlottable(graph);
         if (item->selected() || graph->selected())
         {
             item->setSelected(true);
             graph->setSelection(QCPDataSelection(graph->data()->dataRange()));
         }
     }
}

void MainWindow::mousePress(QCustomPlot* &ui_test)
{
    if (ui_test->xAxis->selectedParts().testFlag(QCPAxis::spAxis))
        ui_test->axisRect()->setRangeDrag(ui_test->xAxis->orientation());
    else if (ui_test->yAxis->selectedParts().testFlag(QCPAxis::spAxis))
        ui_test->axisRect()->setRangeDrag(ui_test->yAxis->orientation());
    else
        ui_test->axisRect()->setRangeDrag(Qt::Horizontal|Qt::Vertical);
}

void MainWindow::mouseWheel(QCustomPlot* &ui_test)
{
    if (ui_test->xAxis->selectedParts().testFlag(QCPAxis::spAxis))
        ui_test->axisRect()->setRangeZoom(ui_test->xAxis->orientation());
    else if (ui_test->yAxis->selectedParts().testFlag(QCPAxis::spAxis))
        ui_test->axisRect()->setRangeZoom(ui_test->yAxis->orientation());
    else
        ui_test->axisRect()->setRangeZoom(Qt::Horizontal|Qt::Vertical);
}

void MainWindow::InitTable(QTableWidget *table, const std::string j_names[], int num){
    QFont tableFont;
    tableFont.setPointSize(8);

    const int item_height = 25;
    const int item_width = 40;
    const int col_0_width = 70;
    const int col_1_width = 70;
    const int col_2_width = 70;
    const int low_height = 28;


    // Horizontal - Column
    for(int i=0; i<3; i++){
        table->insertColumn(i);
        table->setHorizontalHeaderItem(i, new QTableWidgetItem());
        table->horizontalHeaderItem(i)->setTextAlignment(Qt::AlignHCenter|Qt::AlignVCenter|Qt::AlignCenter);
        table->horizontalHeaderItem(i)->setFont(tableFont);
    }
    table->horizontalHeaderItem(0)->setSizeHint(QSize(col_0_width, item_height));
    table->horizontalHeaderItem(1)->setSizeHint(QSize(col_1_width, item_height));
    table->horizontalHeaderItem(2)->setSizeHint(QSize(col_2_width, item_height));
    table->setColumnWidth(0, col_0_width);
    table->setColumnWidth(1, col_1_width);
    table->setColumnWidth(2, col_2_width);
    table->horizontalHeaderItem(0)->setText("Status");
    table->horizontalHeaderItem(1)->setText("Error");
    table->horizontalHeaderItem(2)->setText("Temp");

    // Vertical - Row
    for(int i=0; i<num; i++){
        table->insertRow(i);
        table->setRowHeight(i,low_height);
        table->setVerticalHeaderItem(i, new QTableWidgetItem());
        table->verticalHeaderItem(i)->setText(j_names[i].data());
        table->verticalHeaderItem(i)->setSizeHint(QSize(item_width, item_height));
        table->verticalHeaderItem(i)->setTextAlignment(Qt::AlignHCenter|Qt::AlignVCenter|Qt::AlignCenter);
        table->verticalHeaderItem(i)->setFont(tableFont);
    }

    for(int i=0; i<num; i++){
        for(int j=0; j<3; j++){
            table->setItem(i, j, new QTableWidgetItem());
            table->item(i,j)->setTextAlignment(Qt::AlignHCenter|Qt::AlignVCenter|Qt::AlignCenter);
            table->item(i,j)->setFlags(table->item(i,j)->flags() & ~Qt::ItemIsEditable);
            table->item(i,j)->setFont(tableFont);
        }
    }

    table->setMinimumWidth(item_width + col_0_width + col_1_width + col_2_width + 2);
    table->setMaximumWidth(item_width + col_0_width + col_1_width + col_2_width + 2);
    table->setMinimumHeight(low_height*(num+1) );
    table->setMaximumHeight(low_height*(num+1) );
    //table->resizeColumnsToContents();

    table->setSelectionBehavior(QAbstractItemView::SelectRows);
    table->setSelectionMode(QAbstractItemView::SingleSelection);
    table->horizontalHeader()->setSectionResizeMode(QHeaderView::Fixed);
    table->verticalHeader()->setSectionResizeMode(QHeaderView::Fixed);
}

void MainWindow::on_TW_0_itemSelectionChanged(){
    if(select_working == true)
        return;

    select_working = true;
    lastSelected = ui->TW_0->currentRow();
    ChangeSelectedJoint();
    select_working = false;
}
void MainWindow::ChangeSelectedJoint(){
    ui->LB_SELECTED->setText(QString().sprintf("Selected: %s", JointNameList[lastSelected].data()));
}



// REF ON & OFF ------------------------
void MainWindow::on_BTN_REF_ON_clicked(){
    FILE_LOG(logINFO) << "BTN_REF_ON";

    for(int i=0; i<_NO_OF_MC; i++){     
        MY_CONTROL_FB_GAIN[i] = 10;
        MY_CONTROL_FF_GAIN[i] = 0;
//        _DEV_MC[i].MoveJoints.RefAngleCurrent = _DEV_MC[i].EncoderValue/_DEV_MC[i].PPR;
        _DEV_MC[i].MoveJoints.RefAngleCurrent = _DEV_MC[i].CurrentPosition;
//        cout << "_DEV_MC[i].Reference = " << _DEV_MC[i].Reference << endl;
    }

    double delta_ang[_NO_OF_MC];
    double temp_sum = 0.;
    for(int k=0; k<_NO_OF_MC; k++){
        delta_ang[k] = _DEV_MC[k].MoveJoints.RefAngleCurrent - _DEV_MC[k].CurrentPosition;
        temp_sum += delta_ang[k]*delta_ang[k];
    }
    if(sqrt(temp_sum) < 1.0){
        for(int i=0; i<_NO_OF_MC; i++){
            _DEV_MC[i].RBBoard_ReferenceOutEnable(true);
        }
        sharedData->REF_Enabled = true;
    }else{
        QMessageBox::warning(this, tr("Rainbow Robotics::Warning"), tr("Encoder and Reference should be same !!!"));
    }
}
void MainWindow::on_BTN_REF_OFF_clicked(){
    FILE_LOG(logINFO) << "BTN_REF_OFF";
    for(int i=0; i<_NO_OF_MC; i++){
        _DEV_MC[i].RBBoard_ReferenceOutEnable(false);
    }
    sharedData->REF_Enabled = false;
}
// --------------------------------------


// CAN CHECK & FIND HOME POS ------------
void MainWindow::on_BTN_CAN_CHECK_clicked(){
    FILE_LOG(logINFO) << "BTN_CAN_CHECK";

    on_BTN_IMU_ENABLE_clicked();

    for(int i=0; i<_NO_OF_MC; i++){
        _DEV_MC[i].RBBoard_CANCheck(2);
//        for(int j=0; j<MOTOR_2CH; j++){
//            sharedData->ENCODER[i][j].BoardConnection = _DEV_MC[i].ConnectionStatus;
//        }
    }

    _DEV_MC[HRR].RBFOC_SetGain_POS(200,0,600, 0);
    _DEV_MC[HRP].RBFOC_SetGain_POS(200,0,600, 0);
    _DEV_MC[HRK].RBFOC_SetGain_POS(200,0,600, 0);
    _DEV_MC[HLR].RBFOC_SetGain_POS(200,0,600, 0);
    _DEV_MC[HLP].RBFOC_SetGain_POS(200,0,600, 0);
    _DEV_MC[HLK].RBFOC_SetGain_POS(200,0,600, 0);
    _DEV_MC[FRR].RBFOC_SetGain_POS(200,0,600, 0);
    _DEV_MC[FRP].RBFOC_SetGain_POS(200,0,600, 0);
    _DEV_MC[FRK].RBFOC_SetGain_POS(200,0,600, 0);
    _DEV_MC[FLR].RBFOC_SetGain_POS(200,0,600, 0);
    _DEV_MC[FLP].RBFOC_SetGain_POS(200,0,600, 0);
    _DEV_MC[FLK].RBFOC_SetGain_POS(200,0,600, 0);

    sharedData->plot_timer_zero = true;
    sharedData->plot_play_ON = true;

}

void MainWindow::on_BTN_FIND_HOME_clicked(){
    FILE_LOG(logINFO) << "BTN_FIND_HOME";
    for(int i=0; i<_NO_OF_MC; i++){
        _DEV_MC[i].RBBoard_ReferenceOutEnable(false);
    }
    sharedData->REF_Enabled = false;

    for(int i=0; i<_NO_OF_MC; i++){
        for(int j=0; j<_DEV_MC[i].MOTOR_CHANNEL; j++){
            sharedData->JointReference[i][j] = 0.0;
            _DEV_MC[i].Reference = 0.0;
            _DEV_MC[i].MoveJoints.RefAngleCurrent = 0.0;
            _DEV_MC[i].RB_InitControl();
        }
    }
}

void MainWindow::on_BTN_RESET_PROCESS_1_clicked(){
    if(ui->radioButton_BASIC_CH->isChecked()) {
        int ch =  ui->LE_BASIC_CHNUM->text().toInt();
        _DEV_MC[ch].RB_PVL_RESET();
        FILE_LOG(logSUCCESS) << "CH : " << ch << ", RESET_ALL_PVL!";
    }
    else if(ui->radioButton_BASIC_ALL->isChecked()) {
        for(int i=0; i<_NO_OF_MC; i++){
            _DEV_MC[i].RB_PVL_RESET();
            FILE_LOG(logSUCCESS) << "CH : " << i << ", RESET_ALL_PVL!";
        }
    }
}

void MainWindow::on_BTN_RESET_PROCESS_2_clicked(){
    if(ui->radioButton_BASIC_CH->isChecked()) {
        int ch =  ui->LE_BASIC_CHNUM->text().toInt();
        _DEV_MC[ch].RB_SAVE_MU_DISK_OFFSET();
        FILE_LOG(logSUCCESS) << "CH : " << ch << ", SAVE_DISK_OFFSET!";
    }
    else if(ui->radioButton_BASIC_ALL->isChecked()) {
        for(int i=0; i<_NO_OF_MC; i++){
            _DEV_MC[i].RB_SAVE_MU_DISK_OFFSET();
            FILE_LOG(logSUCCESS) << "CH : " << i << ", SAVE_DISK_OFFSET!";
        }
    }
}

void MainWindow::on_BTN_RESET_PROCESS_3_clicked(){
    if(ui->radioButton_BASIC_CH->isChecked()) {
        int ch =  ui->LE_BASIC_CHNUM->text().toInt();
        _DEV_MC[ch].RBHome_SetHomeZeroOffset(0, 1);
        FILE_LOG(logSUCCESS) << "CH : " << ch << ", DISK_OFFSET_ZERO!";
    }
    else if(ui->radioButton_BASIC_ALL->isChecked()) {
        for(int i=0; i<_NO_OF_MC; i++){
            _DEV_MC[i].RBHome_SetHomeZeroOffset(0, 1);
            FILE_LOG(logSUCCESS) << "CH : " << i << ", DISK_OFFSET_ZERO!";
        }
    }
}

void MainWindow::on_BTN_JOY_CONNECT_clicked()
{
    if(joy->connection==false)
    {
        joy->ConnectJoy(ui->LE_JOYNAME->text());
    }
}

void MainWindow::on_BTN_JOY_DISCONNECT_clicked()
{
    ui->CB_Jcon->setChecked(false);
    joy->DisconnectJoy();
}

void MainWindow::on_BTN_IMU_SET_clicked()
{
    sharedData->alpha_IMUFilter = ui->LE_IMUFilter_alpha->text().toDouble();
    sharedData->frq_IMUFilter = ui->LE_IMUFilter->text().toDouble();

    sharedData->roll_ofs = ui->LE_SENSOR_CIMU_ROLL_OFS->text().toDouble();
    sharedData->pitch_ofs = ui->LE_SENSOR_CIMU_PITCH_OFS->text().toDouble();

    //sharedData->btn_IMUFilter = ui->CB_IMU_UseFilter->isChecked();

    if(ui->CB_isJointFilter->isChecked()){
        sharedData->isJointFilter = true;
        sharedData->frq_JointFilter = ui->LE_JointFilter->text().toDouble();
        FILE_LOG(logSUCCESS) << "Joint Filter ON. Cutoff : " << sharedData->frq_JointFilter << "Hz";
    }
    else{sharedData->isJointFilter = false;}

    sharedData->frq_JointFilter = ui->LE_JointFilter->text().toDouble();
}

void MainWindow::on_BTN_IMU_ENABLE_clicked()
{
    on_BTN_IMU_SET_clicked();
    _DEV_IMU.RBOpenPort(B460800);
    sharedData->alpha_IMUFilter = ui->LE_IMUFilter_alpha->text().toDouble();
    sharedData->frq_IMUFilter = ui->LE_IMUFilter->text().toDouble();
    //sharedData->btn_IMUFilter = ui->CB_IMU_UseFilter->isChecked();
    FILE_LOG(logINFO) << "CMD: DAEMON_SENSOR_IMU_ENABLE";
}

void MainWindow::on_BTN_IMU_NULL_clicked()
{

//    char txData = 0x12;
//    _DEV_IMU.RBWritePort(&txData, 1, 1);

//    RBCAN_MB mb;
//    mb.channel = 0;
//    mb.id = 5;
//    mb.data[0] = 3;
//    mb.dlc = 1;
//    canHandler->RBCAN_WriteData(mb);

    FILE_LOG(logINFO) << "CMD: DAEMON_SENSOR_IMU_NULLING";
}

void MainWindow::on_BTN_IMU_RESET_clicked()
{
//    char txData = 0x10;
//    _DEV_IMU.RBWritePort(&txData, 1, 1);
//    FILE_LOG(logINFO) << "CMD: DAEMON_SENSOR_IMU_RESET";

//    sharedData->IMU[0].RESET_FLAG = true;
}

void MainWindow::on_BTN_IMU_RESET0_clicked()
{
//    char txData = 0x11;
//    _DEV_IMU.RBWritePort(&txData, 1, 1);
//    FILE_LOG(logINFO) << "CMD: DAEMON_SENSOR_IMU_RESET0";
}

void MainWindow::on_BTN_IMU_ALPHA_clicked()
{
    char txData = 0xA1;
    _DEV_IMU.RBWritePort(&txData, 1, 1);
    FILE_LOG(logINFO) << "CMD: DAEMON_SENSOR_IMU_0.001";
}

void MainWindow::on_BTN_IMU_ALPHA_2_clicked()
{
    char txData = 0xB0;
    _DEV_IMU.RBWritePort(&txData, 1, 1);
    FILE_LOG(logINFO) << "CMD: DAEMON_SENSOR_IMU_0.015";
}

void MainWindow::on_BTN_READYPOS_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = READYPOS;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0]= NoTask;
    sharedData->COMMAND = cmd.COMMAND_DATA;
    sharedData->NEWCOMMAND = true;
}

void MainWindow::on_BTN_MOUNTPOS_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = MOUNTPOS;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0]= NoTask;
    sharedData->COMMAND = cmd.COMMAND_DATA;
    sharedData->NEWCOMMAND = true;
}

void MainWindow::on_BTN_GROUNDPOS_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = GROUNDPOS;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0]= NoTask;
    sharedData->COMMAND = cmd.COMMAND_DATA;
    sharedData->NEWCOMMAND = true;
}

void MainWindow::on_BTN_GREADYPOS_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = GREADYPOS;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0]= NoTask;
    sharedData->COMMAND = cmd.COMMAND_DATA;
    sharedData->NEWCOMMAND = true;
}

void MainWindow::on_BTN_HOMMINGPOS_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = HOMMINGPOS;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0]= NoTask;
    sharedData->COMMAND = cmd.COMMAND_DATA;
    sharedData->NEWCOMMAND = true;
}

void MainWindow::on_BTN_BACKFLIP_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = MOTION_BACKFLIP;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0]= NoTask;
    sharedData->COMMAND = cmd.COMMAND_DATA;
    sharedData->NEWCOMMAND = true;
}

void MainWindow::on_BTN_BACKFLIP_H_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = MOTION_BACKFLIP_H;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0]= NoTask;
    sharedData->COMMAND = cmd.COMMAND_DATA;
    sharedData->NEWCOMMAND = true;
}

void MainWindow::on_BTN_FALLPOS_clicked()
{
    sharedData->isFalled = true;
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = MOTION_FALLING;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0]= NoTask;
    sharedData->COMMAND = cmd.COMMAND_DATA;
    sharedData->NEWCOMMAND = true;
}

void MainWindow::on_BTN_FALLRECOVER_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = MOTION_FALLRECOVER;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0]= NoTask;
    sharedData->COMMAND = cmd.COMMAND_DATA;
    sharedData->NEWCOMMAND = true;
}

void MainWindow::on_BTN_JUMP_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = MOTION_JUMP;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0]= NoTask;
    sharedData->COMMAND = cmd.COMMAND_DATA;
    sharedData->NEWCOMMAND = true;
}

void MainWindow::on_BTN_OFF_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = MOTOR_OFF;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0]= NoTask;
    sharedData->COMMAND = cmd.COMMAND_DATA;
    sharedData->NEWCOMMAND = true;
}

void MainWindow::on_BTN_STOP_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = MOTION_STOP;
    sharedData->COMMAND = cmd.COMMAND_DATA;
    sharedData->NEWCOMMAND = true;
    sharedData->isStopping = true;
}


void MainWindow::on_BTN_DEMO_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = MOTION_START;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0]= DemoMotion;
    sharedData->COMMAND = cmd.COMMAND_DATA;
    sharedData->NEWCOMMAND = true;

    BasicSetup();

    sharedData->demo_variable[0] = ui->LE_demo_t->text().toDouble();
    sharedData->demo_variable[1] = ui->LE_demo_x->text().toDouble();
    sharedData->demo_variable[2] = ui->LE_demo_y->text().toDouble();
    sharedData->demo_variable[3] = ui->LE_demo_z->text().toDouble();
    sharedData->demo_variable[4] = ui->LE_demo_R->text().toDouble();
    sharedData->demo_variable[5] = ui->LE_demo_P->text().toDouble();
    sharedData->demo_variable[6] = ui->LE_demo_Y->text().toDouble();

}

void MainWindow::on_BTN_OneLegTest_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = MOTION_START;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0]=OneLegTest;
    sharedData->COMMAND = cmd.COMMAND_DATA;
    sharedData->NEWCOMMAND = true;

    BasicSetup();
    LegControlSetup();

    for(int i=0; i<4; i++)
    {
        sharedData->idx_legcon[i] = false;
    }
}

void MainWindow::on_BTN_OneLegTest_ON_clicked()
{
    if(sharedData->COMMAND.USER_PARA_CHAR[0] == OneLegTest)
    {
        FILE_LOG(logSUCCESS) << "Feedback ON";
        sharedData->OneLegTest_ON = true;
        LegControlSetup();

        if(ui->CB_isJointFilter->isChecked()){
            sharedData->isJointFilter = true;
            sharedData->frq_JointFilter = ui->LE_JointFilter->text().toDouble();
        }
        else{sharedData->isJointFilter = false;}

        if(ui->CB_legcon_HR->isChecked()){
            sharedData->idx_legcon[0] = true;
            sharedData->pf_ref[0+0] = sharedData->pf_ref_init[0+0] + ui->footref_x->text().toDouble();
            sharedData->pf_ref[0+1] = sharedData->pf_ref_init[0+1] + ui->footref_y->text().toDouble();
            sharedData->pf_ref[0+2] = sharedData->pf_ref_init[0+2] + ui->footref_z->text().toDouble();
        }
        else
            sharedData->idx_legcon[0] = false;

        if(ui->CB_legcon_HL->isChecked()){
            sharedData->idx_legcon[1] = true;
            sharedData->pf_ref[3+0] = sharedData->pf_ref_init[3+0] + ui->footref_x->text().toDouble();
            sharedData->pf_ref[3+1] = sharedData->pf_ref_init[3+1] + ui->footref_y->text().toDouble();
            sharedData->pf_ref[3+2] = sharedData->pf_ref_init[3+2] + ui->footref_z->text().toDouble();
        }
        else
            sharedData->idx_legcon[1] = false;

        if(ui->CB_legcon_FR->isChecked()){
            sharedData->idx_legcon[2] = true;
            sharedData->pf_ref[6+0] = sharedData->pf_ref_init[6+0] + ui->footref_x->text().toDouble();
            sharedData->pf_ref[6+1] = sharedData->pf_ref_init[6+1] + ui->footref_y->text().toDouble();
            sharedData->pf_ref[6+2] = sharedData->pf_ref_init[6+2] + ui->footref_z->text().toDouble();
        }
        else
            sharedData->idx_legcon[2] = false;

        if(ui->CB_legcon_FL->isChecked()){
            sharedData->idx_legcon[3] = true;
            sharedData->pf_ref[9+0] = sharedData->pf_ref_init[9+0] + ui->footref_x->text().toDouble();
            sharedData->pf_ref[9+1] = sharedData->pf_ref_init[9+1] + ui->footref_y->text().toDouble();
            sharedData->pf_ref[9+2] = sharedData->pf_ref_init[9+2] + ui->footref_z->text().toDouble();
        }
        else
            sharedData->idx_legcon[3] = false;
    }
    else
        FILE_LOG(logWARNING) << "Not OneLegTest Mode.";
}

void MainWindow::on_BTN_OneLegTest_OFF_clicked()
{
    if(sharedData->COMMAND.USER_PARA_CHAR[0] == OneLegTest)
    {
        FILE_LOG(logSUCCESS) << "Feedback OFF";
        sharedData->OneLegTest_OFF = true;
        for(int i=0; i<4; i++)
            sharedData->idx_legcon[i] = false;
    }
    else
        FILE_LOG(logWARNING) << "Not OneLegTest Mode.";
}



void MainWindow::on_BTN_MG_timerstop_clicked()
{
    if(sharedData->plot_play_ON == true)
        sharedData->plot_play_ON = false;
    else {
        sharedData->plot_play_ON = true;
    }
}

void MainWindow::on_BTN_DANCE01_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = MOTION_START;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0]= Dance01;
    sharedData->COMMAND = cmd.COMMAND_DATA;
    sharedData->NEWCOMMAND = true;

    BasicSetup();
}

//int  enc_request_time_array[MAX_MC] = {
//    6,6,6,
//    7,7,7,
//    8,8,8,
//    9,9,9
//};

void MainWindow::on_BTN_RESET_PROCESS_4_clicked()
{
//    for(int i=0; i<_NO_OF_MC; i++){
//        _DEV_MC[i].RB_Encoder_RequestTime(enc_request_time_array[i]);
//    }
}

void MainWindow::on_BTN_RESET_PROCESS_5_clicked()
{
    if(ui->radioButton_BASIC_CH->isChecked()) {
        int ch =  ui->LE_BASIC_CHNUM->text().toInt();
        _DEV_MC[ch].RB_DQAlign();
        FILE_LOG(logSUCCESS) << "CH : " << ch << ", DQ Align!";
    }
    else if(ui->radioButton_BASIC_ALL->isChecked()) {
        for(int i=0; i<_NO_OF_MC; i++){
            _DEV_MC[i].RB_DQAlign();
            FILE_LOG(logSUCCESS) << "CH : " << i << ", DQ Align!";
        }
    }
}

void MainWindow::on_BTN_RESET_PROCESS_6_clicked()
{
    if(ui->radioButton_BASIC_CH->isChecked()) {
        int ch =  ui->LE_BASIC_CHNUM->text().toInt();
        _DEV_MC[ch].RB_CurrentNulling();
        FILE_LOG(logSUCCESS) << "CH : " << ch << ", Current Nulling!";
    }
    else if(ui->radioButton_BASIC_ALL->isChecked()) {
        for(int i=0; i<_NO_OF_MC; i++){
            _DEV_MC[i].RB_CurrentNulling();
            FILE_LOG(logSUCCESS) << "CH : " << i << ", Current Nulling!";
        }
    }
}

void MainWindow::on_BTN_FOC_GAINSET_clicked()
{
    int kp = ui->LE_FOC_KP->text().toInt();
    int ki = ui->LE_FOC_KI->text().toInt();

    if(ui->radioButton_BASIC_CH->isChecked()) {
        int ch =  ui->LE_BASIC_CHNUM->text().toInt();
        _DEV_MC[ch].RBFOC_SetGain(kp,ki, 0);
    }
    else if(ui->radioButton_BASIC_ALL->isChecked()) {
        for(int i=0; i<_NO_OF_MC; i++){
            _DEV_MC[i].RBFOC_SetGain(kp,ki, 0);
        }
    }
}

void MainWindow::on_BTN_FOC_GAINREQ_clicked()
{
    int kp = ui->LE_FOC_KP->text().toInt();
    int ki = ui->LE_FOC_KI->text().toInt();

    if(ui->radioButton_BASIC_CH->isChecked()) {
        int ch =  ui->LE_BASIC_CHNUM->text().toInt();
        _DEV_MC[ch].RBFOC_SetGain(kp,ki, 2);
    }
    else if(ui->radioButton_BASIC_ALL->isChecked()) {
        for(int i=0; i<_NO_OF_MC; i++){
            _DEV_MC[i].RBFOC_SetGain(kp,ki, 2);
        }
    }
}

void MainWindow::on_BTN_POS_GAINSET_clicked()
{
    int kp = ui->LE_POS_KP->text().toInt();
    int kd = ui->LE_POS_KD->text().toInt();
    int ki = ui->LE_POS_KI->text().toInt();

    if(ui->radioButton_BASIC_CH->isChecked()) {
        int ch =  ui->LE_BASIC_CHNUM->text().toInt();
        _DEV_MC[ch].RBFOC_SetGain_POS(kp,ki,kd, 0);
    }
    else if(ui->radioButton_BASIC_ALL->isChecked()) {
        for(int i=0; i<_NO_OF_MC; i++){
            _DEV_MC[i].RBFOC_SetGain_POS(kp,ki,kd, 0);
        }
    }
}

void MainWindow::on_BTN_POS_GAINREQ_clicked()
{
    int kp = ui->LE_POS_KP->text().toInt();
    int kd = ui->LE_POS_KD->text().toInt();
    int ki = ui->LE_POS_KI->text().toInt();

    if(ui->radioButton_BASIC_CH->isChecked()) {
        int ch =  ui->LE_BASIC_CHNUM->text().toInt();
        _DEV_MC[ch].RBFOC_SetGain_POS(kp,ki,kd, 2);
    }
    else if(ui->radioButton_BASIC_ALL->isChecked()) {
        for(int i=0; i<_NO_OF_MC; i++){
            _DEV_MC[i].RBFOC_SetGain_POS(kp,ki,kd, 2);
        }
    }
}

void MainWindow::on_BTN_MG_Trot_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = MOTION_START;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0]=MPC;
    sharedData->COMMAND = cmd.COMMAND_DATA;
    sharedData->NEWCOMMAND = true;

    BasicSetup();
    LegControlSetup();
    MPCGainUpdate();
}

void MainWindow::on_BTN_MG_stepping_clicked()
{
}

void MainWindow::on_BTN_MPCgainset_clicked()
{
    MPCGainUpdate();
}

void MainWindow::on_BTN_MG_timerstop_2_clicked()
{
}

void MainWindow::on_BTN_ENC_CHECK_clicked()
{
    double margin = 15.0;
    double res = 360.0/11.0;
    int state;
    if(ui->radioButton_ENC_CHECK_LIFT->isChecked()) {
        state = 0;
        FILE_LOG(logINFO) << "BTN_ENC_CHECK  -  Lift position";
        for(int i=0; i<12; i++) {
            if(_DEV_MC[i].CurrentPosition > _DEV_MC[i].homeJointLift + margin) {
                _DEV_MC[i].homeJointOffset_mult -= res;
            }
            else if(_DEV_MC[i].CurrentPosition < _DEV_MC[i].homeJointLift - margin) {
                _DEV_MC[i].homeJointOffset_mult += res;
            }
        }
    }
    else if(ui->radioButton_ENC_CHECK_READY->isChecked()) {
        state = 1;
        FILE_LOG(logINFO) << "BTN_ENC_CHECK  -  Ready position";
        for(int i=0; i<12; i++) {
            if(_DEV_MC[i].CurrentPosition > _DEV_MC[i].homeJointReady + margin) {
                _DEV_MC[i].homeJointOffset_mult -= res;
            }
            else if(_DEV_MC[i].CurrentPosition < _DEV_MC[i].homeJointReady - margin) {
                _DEV_MC[i].homeJointOffset_mult += res;
            }
        }
    }
    else if(ui->radioButton_ENC_CHECK_GROUND->isChecked()) {
        state = 2;
        FILE_LOG(logINFO) << "BTN_ENC_CHECK  -  Ground position";
        for(int i=0; i<12; i++) {
            if(_DEV_MC[i].CurrentPosition > _DEV_MC[i].homeJointGround + margin) {
                _DEV_MC[i].homeJointOffset_mult -= res;
            }
            else if(_DEV_MC[i].CurrentPosition < _DEV_MC[i].homeJointGround - margin) {
                _DEV_MC[i].homeJointOffset_mult += res;
            }
        }
    }
    else if(ui->radioButton_ENC_CHECK_HOMMING->isChecked()) {
        state = 3;
        FILE_LOG(logINFO) << "BTN_ENC_CHECK  -  Ground position";
        for(int i=0; i<12; i++) {
            if(_DEV_MC[i].CurrentPosition > _DEV_MC[i].homeJointOffset + margin) {
                _DEV_MC[i].homeJointOffset_mult -= res;
            }
            else if(_DEV_MC[i].CurrentPosition < _DEV_MC[i].homeJointOffset - margin) {
                _DEV_MC[i].homeJointOffset_mult += res;
            }
        }
    }
}

void MainWindow::on_BTN_JOG_ON_clicked()
{
    Jog_state ^= 1;
    if(Jog_state == 1) ui->BTN_JOG_ON->setStyleSheet("QPushButton{background-color:brown}");
    else ui->BTN_JOG_ON->setStyleSheet("QPushButton{background-color:white}");
}

void MainWindow::on_BTN_JOG_SPEED_1_clicked()
{
    Jog_speed_state = 0;
}

void MainWindow::on_BTN_JOG_SPEED_2_clicked()
{
    Jog_speed_state = 1;
}

void MainWindow::on_BTN_JOG_SPEED_3_clicked()
{
    Jog_speed_state = 2;
}

void MainWindow::on_BTN_JOG_SPEED_4_clicked()
{
    Jog_speed_state = 3;
}

void MainWindow::on_BTN_JOG_MODE_1_clicked()
{
    Jog_mode = 0;
}

void MainWindow::on_BTN_JOG_MODE_2_clicked()
{
    Jog_mode = 1;
}

static int Jog_released_flag = 0;
void MainWindow::on_BTN_JOG_UP_pressed()
{
//    if(Jog_mode == 1) {
//        Jog_released_flag = 0;
//        USER_COMMAND cmd;
//        cmd.COMMAND_DATA.USER_COMMAND = JOGMODE;
//        cmd.COMMAND_DATA.USER_PARA_CHAR[0]=Jog_mode;
//        cmd.COMMAND_DATA.USER_PARA_CHAR[1]=sharedData->jog_joint;
//        cmd.COMMAND_DATA.USER_PARA_DOUBLE[0]= Jog_JointMov;
//        sharedData->COMMAND = cmd.COMMAND_DATA;
//        sharedData->NEWCOMMAND = true;
//    }
}

void MainWindow::on_BTN_JOG_UP_released()
{
//    if(Jog_released_flag == 0) {
//        Jog_released_flag = 1;
//        USER_COMMAND cmd;
//        cmd.COMMAND_DATA.USER_COMMAND = NO_ACT;
//        cmd.COMMAND_DATA.USER_PARA_CHAR[0]=Jog_mode;
//        cmd.COMMAND_DATA.USER_PARA_CHAR[1]=0;
//        cmd.COMMAND_DATA.USER_PARA_DOUBLE[0]= 0;
//        sharedData->COMMAND = cmd.COMMAND_DATA;
//        sharedData->NEWCOMMAND = true;
//    }
}

void MainWindow::on_BTN_JOG_DOWN_pressed()
{
//    if(Jog_mode == 1) {
//        Jog_released_flag = 0;
//        USER_COMMAND cmd;
//        cmd.COMMAND_DATA.USER_COMMAND = JOGMODE;
//        cmd.COMMAND_DATA.USER_PARA_CHAR[0]=Jog_mode;
//        cmd.COMMAND_DATA.USER_PARA_CHAR[1]=sharedData->jog_joint;
//        cmd.COMMAND_DATA.USER_PARA_DOUBLE[0]= -Jog_JointMov;
//        sharedData->COMMAND = cmd.COMMAND_DATA;
//        sharedData->NEWCOMMAND = true;
//    }
}

void MainWindow::on_BTN_JOG_DOWN_released()
{
//    if(Jog_released_flag == 0) {
//        Jog_released_flag = 1;
//        USER_COMMAND cmd;
//        cmd.COMMAND_DATA.USER_COMMAND = NO_ACT;
//        cmd.COMMAND_DATA.USER_PARA_CHAR[0]=Jog_mode;
//        cmd.COMMAND_DATA.USER_PARA_CHAR[1]=0;
//        cmd.COMMAND_DATA.USER_PARA_DOUBLE[0]= 0;
//        sharedData->COMMAND = cmd.COMMAND_DATA;
//        sharedData->NEWCOMMAND = true;
//    }
}


void MainWindow::on_BTN_JOG_UP_clicked()
{
    if(Jog_mode == 0) {
        Jog_released_flag = 0;
        USER_COMMAND cmd;
        cmd.COMMAND_DATA.USER_COMMAND = JOGMODE;
        cmd.COMMAND_DATA.USER_PARA_CHAR[0]=Jog_mode;
        cmd.COMMAND_DATA.USER_PARA_DOUBLE[0]= Jog_JointMov;
        sharedData->COMMAND = cmd.COMMAND_DATA;
        sharedData->NEWCOMMAND = true;
    }
}

void MainWindow::on_BTN_JOG_DOWN_clicked()
{
    if(Jog_mode == 0) {
        Jog_released_flag = 0;
        USER_COMMAND cmd;
        cmd.COMMAND_DATA.USER_COMMAND = JOGMODE;
        cmd.COMMAND_DATA.USER_PARA_CHAR[0]=Jog_mode;
        cmd.COMMAND_DATA.USER_PARA_DOUBLE[0]= -Jog_JointMov;
        sharedData->COMMAND = cmd.COMMAND_DATA;
        sharedData->NEWCOMMAND = true;
    }
}

void MainWindow::on_BTN_TROTTEST_SET_clicked()
{
    sharedData->trot_variable_int[1] = ui->CB_TROTTEST_CONTACTFLAG->isChecked();
    sharedData->trot_variable_int[2] = ui->radioButton_TROTTEST_MODE2->isChecked();
    sharedData->control_variable_double[0] = ui->LE_TROTTEST_hpf_frq->text().toDouble();

    sharedData->Kp[0] = ui->Kp_x->text().toDouble();
    sharedData->Kp[1] = ui->Kp_y->text().toDouble();
    sharedData->Kp[2] = ui->Kp_z->text().toDouble();

    sharedData->Kd[0] = ui->Kd_x->text().toDouble();
    sharedData->Kd[1] = ui->Kd_y->text().toDouble();
    sharedData->Kd[2] = ui->Kd_z->text().toDouble();

    sharedData->Kp_j[0] = ui->Kp_r->text().toDouble();
    sharedData->Kp_j[1] = ui->Kp_p->text().toDouble();
    sharedData->Kp_j[2] = ui->Kp_k->text().toDouble();

    sharedData->Kd_j[0] = ui->Kd_r->text().toDouble();
    sharedData->Kd_j[1] = ui->Kd_p->text().toDouble();
    sharedData->Kd_j[2] = ui->Kd_k->text().toDouble();

    sharedData->Cmax = ui->LE_Cmax->text().toDouble();

    if(ui->CB_TASK_IMU_ENABLE->isChecked()) {
        sharedData->control_variable_double[1] = ui->LE_COMCON_kp_roll->text().toDouble();
        sharedData->control_variable_double[2] = ui->LE_COMCON_kd_roll->text().toDouble();
        sharedData->control_variable_double[3] = ui->LE_COMCON_kp_pitch->text().toDouble();
        sharedData->control_variable_double[4] = ui->LE_COMCON_kd_pitch->text().toDouble();
        sharedData->control_variable_double[5] = ui->LE_COMCON_kp_yaw->text().toDouble();
        sharedData->control_variable_double[6] = ui->LE_COMCON_kd_yaw->text().toDouble();
    }
    else {
        for(int i=1; i<13; i++) sharedData->control_variable_double[i] = 0;
    }

    sharedData->trot_variable_double[11] = ui->LE_TROTTEST_kp_x->text().toDouble();
    sharedData->trot_variable_double[12] = ui->LE_TROTTEST_kd_x->text().toDouble();
    sharedData->trot_variable_double[13] = ui->LE_TROTTEST_kp_y->text().toDouble();
    sharedData->trot_variable_double[14] = ui->LE_TROTTEST_kd_y->text().toDouble();
    sharedData->trot_variable_double[15] = ui->LE_TROTTEST_kp_z->text().toDouble();
    sharedData->trot_variable_double[16] = ui->LE_TROTTEST_kd_z->text().toDouble();

    sharedData->trot_variable_double[17] = ui->LE_TROTTEST_COM_X->text().toDouble();
    sharedData->trot_variable_double[18] = ui->LE_TROTTEST_COM_Y->text().toDouble();
    sharedData->trot_variable_double[19] = ui->LE_TROTTEST_COM_Z->text().toDouble();

    sharedData->trot_variable_double[20] = ui->LE_TROTTEST_TJUMP->text().toDouble();
    sharedData->trot_variable_double[21] = ui->LE_TROTTEST_OFSJUMP->text().toDouble();

    sharedData->trot_variable_double[22] = ui->LE_TROTTEST_FF_FORCE_X->text().toDouble();
    sharedData->trot_variable_double[23] = ui->LE_TROTTEST_FF_FORCE_Y->text().toDouble();
    sharedData->trot_variable_double[24] = ui->LE_TROTTEST_FF_FORCE_Z->text().toDouble();

    sharedData->trot_variable_double[25] = ui->LE_TROTTEST_ka_x->text().toDouble();
    sharedData->trot_variable_double[26] = ui->LE_TROTTEST_ka_y->text().toDouble();

}

void MainWindow::on_BTN_TROTTEST_READY_clicked()
{
    on_BTN_IMU_RESET_clicked();

    on_BTN_TROTTEST_SET_clicked();

    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = MOTION_START;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0]= TrotTest;
    sharedData->COMMAND = cmd.COMMAND_DATA;
    sharedData->NEWCOMMAND = true;

    if(ui->CB_LEGCON_HR->isChecked()) sharedData->task_legcon[HR] = true;
    else sharedData->task_legcon[HR] = false;
    if(ui->CB_LEGCON_HL->isChecked()) sharedData->task_legcon[HL] = true;
    else sharedData->task_legcon[HL] = false;
    if(ui->CB_LEGCON_FR->isChecked()) sharedData->task_legcon[FR] = true;
    else sharedData->task_legcon[FR] = false;
    if(ui->CB_LEGCON_FL->isChecked()) sharedData->task_legcon[FL] = true;
    else sharedData->task_legcon[FL] = false;

    sharedData->trot_variable_int[0] = 0;
    sharedData->trot_variable_int[1] = ui->CB_TROTTEST_CONTACTFLAG->isChecked();
    //sharedData->trot_variable_int[2] = ui->radioButton_TROTTEST_MODE2->isChecked();

    sharedData->trot_variable_double[0] = ui->LE_TROTTEST_INTERVAL->text().toDouble();
    sharedData->trot_variable_double[1] = ui->LE_TROTTEST_PAUSE->text().toDouble();
    sharedData->trot_variable_double[2] = ui->LE_TROTTEST_FFX->text().toDouble();
    sharedData->trot_variable_double[3] = ui->LE_TROTTEST_FFY->text().toDouble();
    sharedData->trot_variable_double[4] = ui->LE_TROTTEST_FFW->text().toDouble();
    sharedData->trot_variable_double[5] = ui->LE_TROTTEST_ROTCOMP->text().toDouble();
    sharedData->trot_variable_double[6] = ui->LE_TROTTEST_CONTACTTHRES->text().toDouble();
    sharedData->trot_variable_double[7] = ui->LE_TROTTEST_CONTACTOFFSET->text().toDouble();
    sharedData->trot_variable_double[8] = ui->LE_TROTTEST_GCOMP_M->text().toDouble();
    sharedData->trot_variable_double[9] = ui->LE_TROTTEST_GCOMP_S->text().toDouble();
    sharedData->trot_variable_double[10] = ui->LE_TROTTEST_STEPZ->text().toDouble();

    sharedData->trot_variable_double[20] = ui->LE_TROTTEST_TJUMP->text().toDouble();
    sharedData->trot_variable_double[21] = ui->LE_TROTTEST_OFSJUMP->text().toDouble();

    sharedData->trot_variable_double[22] = ui->LE_TROTTEST_FF_FORCE_X->text().toDouble();
    sharedData->trot_variable_double[23] = ui->LE_TROTTEST_FF_FORCE_Y->text().toDouble();
    sharedData->trot_variable_double[24] = ui->LE_TROTTEST_FF_FORCE_Z->text().toDouble();

    sharedData->trot_variable_double[25] = ui->LE_TROTTEST_ka_x->text().toDouble();
    sharedData->trot_variable_double[26] = ui->LE_TROTTEST_ka_y->text().toDouble();


    BasicSetup();
    LegControlSetup();
}

void MainWindow::on_BTN_TROTTEST_START_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0]= TrotTest;
    sharedData->COMMAND = cmd.COMMAND_DATA;
    sharedData->NEWCOMMAND = true;

    if(ui->CB_LEGCON_HR->isChecked()) sharedData->task_legcon[HR] = true;
    else sharedData->task_legcon[HR] = false;
    if(ui->CB_LEGCON_HL->isChecked()) sharedData->task_legcon[HL] = true;
    else sharedData->task_legcon[HL] = false;
    if(ui->CB_LEGCON_FR->isChecked()) sharedData->task_legcon[FR] = true;
    else sharedData->task_legcon[FR] = false;
    if(ui->CB_LEGCON_FL->isChecked()) sharedData->task_legcon[FL] = true;
    else sharedData->task_legcon[FL] = false;

    sharedData->trot_variable_int[0] = 1;
    sharedData->trot_variable_int[1] = ui->CB_TROTTEST_CONTACTFLAG->isChecked();
    //sharedData->trot_variable_int[2] = ui->radioButton_TROTTEST_MODE2->isChecked();

    sharedData->trot_variable_double[0] = ui->LE_TROTTEST_INTERVAL->text().toDouble();
    sharedData->trot_variable_double[1] = ui->LE_TROTTEST_PAUSE->text().toDouble();
    sharedData->trot_variable_double[2] = ui->LE_TROTTEST_FFX->text().toDouble();
    sharedData->trot_variable_double[3] = ui->LE_TROTTEST_FFY->text().toDouble();
    sharedData->trot_variable_double[4] = ui->LE_TROTTEST_FFW->text().toDouble();
    sharedData->trot_variable_double[5] = ui->LE_TROTTEST_ROTCOMP->text().toDouble();
    sharedData->trot_variable_double[6] = ui->LE_TROTTEST_CONTACTTHRES->text().toDouble();
    sharedData->trot_variable_double[7] = ui->LE_TROTTEST_CONTACTOFFSET->text().toDouble();
    sharedData->trot_variable_double[8] = ui->LE_TROTTEST_GCOMP_M->text().toDouble();
    sharedData->trot_variable_double[9] = ui->LE_TROTTEST_GCOMP_S->text().toDouble();
    sharedData->trot_variable_double[10] = ui->LE_TROTTEST_STEPZ->text().toDouble();

    sharedData->trot_variable_double[20] = ui->LE_TROTTEST_TJUMP->text().toDouble();
    sharedData->trot_variable_double[21] = ui->LE_TROTTEST_OFSJUMP->text().toDouble();

    BasicSetup();
}

void MainWindow::on_BTN_TROTTEST_STOP_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0]= TrotTest;
    sharedData->COMMAND = cmd.COMMAND_DATA;
    sharedData->NEWCOMMAND = true;

    sharedData->trot_variable_int[0] = 2;
}

void MainWindow::on_BTN_TROTTEST_ONESTEP_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0]= TrotTest;
    sharedData->COMMAND = cmd.COMMAND_DATA;
    sharedData->NEWCOMMAND = true;

    sharedData->trot_variable_int[0] = 3;
    sharedData->trot_variable_double[0] = ui->LE_TROTTEST_INTERVAL->text().toDouble();
    sharedData->trot_variable_double[1] = ui->LE_TROTTEST_PAUSE->text().toDouble();
    sharedData->trot_variable_double[2] = ui->LE_TROTTEST_FFX->text().toDouble();
    sharedData->trot_variable_double[3] = ui->LE_TROTTEST_FFY->text().toDouble();
    sharedData->trot_variable_double[4] = ui->LE_TROTTEST_FFW->text().toDouble();
    sharedData->trot_variable_double[5] = ui->LE_TROTTEST_ROTCOMP->text().toDouble();
    sharedData->trot_variable_double[6] = ui->LE_TROTTEST_CONTACTTHRES->text().toDouble();
    sharedData->trot_variable_double[7] = ui->LE_TROTTEST_CONTACTOFFSET->text().toDouble();
    sharedData->trot_variable_double[8] = ui->LE_TROTTEST_GCOMP_M->text().toDouble();
    sharedData->trot_variable_double[9] = ui->LE_TROTTEST_GCOMP_S->text().toDouble();
    sharedData->trot_variable_double[10] = ui->LE_TROTTEST_STEPZ->text().toDouble();


    BasicSetup();
}


void MainWindow::on_BTN_HOPTEST_SET_clicked()
{
    sharedData->Kp[0] = ui->Kp_x_2->text().toDouble();
    sharedData->Kp[1] = ui->Kp_y_2->text().toDouble();
    sharedData->Kp[2] = ui->Kp_z_2->text().toDouble();

    sharedData->Kd[0] = ui->Kd_x_2->text().toDouble();
    sharedData->Kd[1] = ui->Kd_y_2->text().toDouble();
    sharedData->Kd[2] = ui->Kd_z_2->text().toDouble();

    sharedData->Kp_j[0] = ui->Kp_r_2->text().toDouble();
    sharedData->Kp_j[1] = ui->Kp_p_2->text().toDouble();
    sharedData->Kp_j[2] = ui->Kp_k_2->text().toDouble();

    sharedData->Kd_j[0] = ui->Kd_r_2->text().toDouble();
    sharedData->Kd_j[1] = ui->Kd_p_2->text().toDouble();
    sharedData->Kd_j[2] = ui->Kd_k_2->text().toDouble();

    sharedData->Cmax = ui->LE_Cmax->text().toDouble();

    if(ui->CB_TASK_IMU_ENABLE->isChecked()) {
        sharedData->control_variable_double[1] = ui->LE_COMCON_kp_roll->text().toDouble();
        sharedData->control_variable_double[2] = ui->LE_COMCON_kd_roll->text().toDouble();
        sharedData->control_variable_double[3] = ui->LE_COMCON_kp_pitch->text().toDouble();
        sharedData->control_variable_double[4] = ui->LE_COMCON_kd_pitch->text().toDouble();
        sharedData->control_variable_double[5] = ui->LE_COMCON_kp_yaw->text().toDouble();
        sharedData->control_variable_double[6] = ui->LE_COMCON_kd_yaw->text().toDouble();
    }
    else {
        for(int i=1; i<13; i++) sharedData->control_variable_double[i] = 0;
    }
}

void MainWindow::on_BTN_HOPTEST_READY_clicked()
{
    on_BTN_IMU_RESET_clicked();

    on_BTN_HOPTEST_SET_clicked();

    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = MOTION_START;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0]= HoppingTest;
    sharedData->COMMAND = cmd.COMMAND_DATA;
    sharedData->NEWCOMMAND = true;

    if(ui->CB_LEGCON_HR->isChecked()) sharedData->task_legcon[HR] = true;
    else sharedData->task_legcon[HR] = false;
    if(ui->CB_LEGCON_HL->isChecked()) sharedData->task_legcon[HL] = true;
    else sharedData->task_legcon[HL] = false;
    if(ui->CB_LEGCON_FR->isChecked()) sharedData->task_legcon[FR] = true;
    else sharedData->task_legcon[FR] = false;
    if(ui->CB_LEGCON_FL->isChecked()) sharedData->task_legcon[FL] = true;
    else sharedData->task_legcon[FL] = false;

    sharedData->hopping_variable_int[0] = 0;

    if(ui->BTN_HOPTEST_M1->isChecked()) {
        sharedData->hopping_variable_double[0] = ui->LE_HOPTEST_INTERVAL_1->text().toDouble();
        sharedData->hopping_variable_double[1] = ui->LE_HOPTEST_INTERVAL_2->text().toDouble();
        sharedData->hopping_variable_double[2] = ui->LE_HOPTEST_INTERVAL_3->text().toDouble();
        sharedData->hopping_variable_double[3] = ui->LE_HOPTEST_INTERVAL_4->text().toDouble();
        sharedData->hopping_variable_double[4] = ui->LE_HOPTEST_INTERVAL_5->text().toDouble();
        sharedData->hopping_variable_double[5] = ui->LE_HOPTEST_PAUSE->text().toDouble();

        sharedData->hopping_variable_double[6+0] = ui->LE_HOPTEST_POS_1->text().toDouble();
        sharedData->hopping_variable_double[6+1] = ui->LE_HOPTEST_POS_2->text().toDouble();
        sharedData->hopping_variable_double[6+2] = ui->LE_HOPTEST_POS_3->text().toDouble();
        sharedData->hopping_variable_double[6+3] = ui->LE_HOPTEST_POS_4->text().toDouble();
        sharedData->hopping_variable_double[6+4] = ui->LE_HOPTEST_POS_5->text().toDouble();
    }
    else {
        sharedData->hopping_variable_double[0] = ui->LE_HOPTEST_INTERVAL_6->text().toDouble();
        sharedData->hopping_variable_double[1] = ui->LE_HOPTEST_INTERVAL_7->text().toDouble();
        sharedData->hopping_variable_double[2] = ui->LE_HOPTEST_INTERVAL_8->text().toDouble();
        sharedData->hopping_variable_double[3] = ui->LE_HOPTEST_INTERVAL_9->text().toDouble();
        sharedData->hopping_variable_double[4] = ui->LE_HOPTEST_INTERVAL_10->text().toDouble();
        sharedData->hopping_variable_double[5] = ui->LE_HOPTEST_PAUSE->text().toDouble();

        sharedData->hopping_variable_double[6+0] = ui->LE_HOPTEST_POS_6->text().toDouble();
        sharedData->hopping_variable_double[6+1] = ui->LE_HOPTEST_POS_7->text().toDouble();
        sharedData->hopping_variable_double[6+2] = ui->LE_HOPTEST_POS_8->text().toDouble();
        sharedData->hopping_variable_double[6+3] = ui->LE_HOPTEST_POS_9->text().toDouble();
        sharedData->hopping_variable_double[6+4] = ui->LE_HOPTEST_POS_10->text().toDouble();
    }


    BasicSetup();
    //LegControlSetup();
}

void MainWindow::on_BTN_HOPTEST_START_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0]= HoppingTest;
    sharedData->COMMAND = cmd.COMMAND_DATA;
    sharedData->NEWCOMMAND = true;

    if(ui->CB_LEGCON_HR->isChecked()) sharedData->task_legcon[HR] = true;
    else sharedData->task_legcon[HR] = false;
    if(ui->CB_LEGCON_HL->isChecked()) sharedData->task_legcon[HL] = true;
    else sharedData->task_legcon[HL] = false;
    if(ui->CB_LEGCON_FR->isChecked()) sharedData->task_legcon[FR] = true;
    else sharedData->task_legcon[FR] = false;
    if(ui->CB_LEGCON_FL->isChecked()) sharedData->task_legcon[FL] = true;
    else sharedData->task_legcon[FL] = false;

    sharedData->hopping_variable_int[0] = 1;

    if(ui->BTN_HOPTEST_M1->isChecked()) {
        sharedData->hopping_variable_double[0] = ui->LE_HOPTEST_INTERVAL_1->text().toDouble();
        sharedData->hopping_variable_double[1] = ui->LE_HOPTEST_INTERVAL_2->text().toDouble();
        sharedData->hopping_variable_double[2] = ui->LE_HOPTEST_INTERVAL_3->text().toDouble();
        sharedData->hopping_variable_double[3] = ui->LE_HOPTEST_INTERVAL_4->text().toDouble();
        sharedData->hopping_variable_double[4] = ui->LE_HOPTEST_INTERVAL_5->text().toDouble();
        sharedData->hopping_variable_double[5] = ui->LE_HOPTEST_PAUSE->text().toDouble();

        sharedData->hopping_variable_double[6+0] = ui->LE_HOPTEST_POS_1->text().toDouble();
        sharedData->hopping_variable_double[6+1] = ui->LE_HOPTEST_POS_2->text().toDouble();
        sharedData->hopping_variable_double[6+2] = ui->LE_HOPTEST_POS_3->text().toDouble();
        sharedData->hopping_variable_double[6+3] = ui->LE_HOPTEST_POS_4->text().toDouble();
        sharedData->hopping_variable_double[6+4] = ui->LE_HOPTEST_POS_5->text().toDouble();
    }
    else {
        sharedData->hopping_variable_double[0] = ui->LE_HOPTEST_INTERVAL_6->text().toDouble();
        sharedData->hopping_variable_double[1] = ui->LE_HOPTEST_INTERVAL_7->text().toDouble();
        sharedData->hopping_variable_double[2] = ui->LE_HOPTEST_INTERVAL_8->text().toDouble();
        sharedData->hopping_variable_double[3] = ui->LE_HOPTEST_INTERVAL_9->text().toDouble();
        sharedData->hopping_variable_double[4] = ui->LE_HOPTEST_INTERVAL_10->text().toDouble();
        sharedData->hopping_variable_double[5] = ui->LE_HOPTEST_PAUSE->text().toDouble();

        sharedData->hopping_variable_double[6+0] = ui->LE_HOPTEST_POS_6->text().toDouble();
        sharedData->hopping_variable_double[6+1] = ui->LE_HOPTEST_POS_7->text().toDouble();
        sharedData->hopping_variable_double[6+2] = ui->LE_HOPTEST_POS_8->text().toDouble();
        sharedData->hopping_variable_double[6+3] = ui->LE_HOPTEST_POS_9->text().toDouble();
        sharedData->hopping_variable_double[6+4] = ui->LE_HOPTEST_POS_10->text().toDouble();
    }

    BasicSetup();
}

void MainWindow::on_BTN_HOPTEST_STOP_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0]= HoppingTest;
    sharedData->COMMAND = cmd.COMMAND_DATA;
    sharedData->NEWCOMMAND = true;

    if(ui->CB_LEGCON_HR->isChecked()) sharedData->task_legcon[HR] = true;
    else sharedData->task_legcon[HR] = false;
    if(ui->CB_LEGCON_HL->isChecked()) sharedData->task_legcon[HL] = true;
    else sharedData->task_legcon[HL] = false;
    if(ui->CB_LEGCON_FR->isChecked()) sharedData->task_legcon[FR] = true;
    else sharedData->task_legcon[FR] = false;
    if(ui->CB_LEGCON_FL->isChecked()) sharedData->task_legcon[FL] = true;
    else sharedData->task_legcon[FL] = false;

    sharedData->hopping_variable_int[0] = 2;
}

void MainWindow::on_BTN_CONTROLLER_SET_clicked()
{
//    if(ui->CB_TASK_IMU_ENABLE->isChecked()) {
//        sharedData->control_variable_double[1] = ui->LE_COMCON_kp_roll->text().toDouble();
//        sharedData->control_variable_double[2] = ui->LE_COMCON_kd_roll->text().toDouble();
//        sharedData->control_variable_double[3] = ui->LE_COMCON_kp_pitch->text().toDouble();
//        sharedData->control_variable_double[4] = ui->LE_COMCON_kd_pitch->text().toDouble();
//        sharedData->control_variable_double[5] = ui->LE_COMCON_kp_yaw->text().toDouble();
//        sharedData->control_variable_double[6] = ui->LE_COMCON_kd_yaw->text().toDouble();
//    }
//    else {
//        for(int i=1; i<13; i++) sharedData->control_variable_double[i] = 0;
//    }
}

void MainWindow::on_BTN_INCGAIN_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = INCGAIN;
    sharedData->COMMAND = cmd.COMMAND_DATA;
    sharedData->NEWCOMMAND = true;
}



void MainWindow::on_BTN_SIM_RESET_clicked()
{
    sharedData->RaisimResetFlag = true;
}



void MainWindow::on_BTN_FLYINGTROT_SET_clicked()
{
    sharedData->control_variable_double[0] = ui->LE_TROTTEST_hpf_frq->text().toDouble();

    sharedData->Kp[0] = ui->Kp_x->text().toDouble();
    sharedData->Kp[1] = ui->Kp_y->text().toDouble();
    sharedData->Kp[2] = ui->Kp_z->text().toDouble();

    sharedData->Kd[0] = ui->Kd_x->text().toDouble();
    sharedData->Kd[1] = ui->Kd_y->text().toDouble();
    sharedData->Kd[2] = ui->Kd_z->text().toDouble();

    sharedData->Kp_j[0] = ui->Kp_r->text().toDouble();
    sharedData->Kp_j[1] = ui->Kp_p->text().toDouble();
    sharedData->Kp_j[2] = ui->Kp_k->text().toDouble();

    sharedData->Kd_j[0] = ui->Kd_r->text().toDouble();
    sharedData->Kd_j[1] = ui->Kd_p->text().toDouble();
    sharedData->Kd_j[2] = ui->Kd_k->text().toDouble();

    sharedData->Cmax = ui->LE_Cmax->text().toDouble();

    if(ui->CB_TASK_IMU_ENABLE->isChecked()) {
        sharedData->control_variable_double[1] = ui->LE_COMCON_kp_roll->text().toDouble();
        sharedData->control_variable_double[2] = ui->LE_COMCON_kd_roll->text().toDouble();
        sharedData->control_variable_double[3] = ui->LE_COMCON_kp_pitch->text().toDouble();
        sharedData->control_variable_double[4] = ui->LE_COMCON_kd_pitch->text().toDouble();
        sharedData->control_variable_double[5] = ui->LE_COMCON_kp_yaw->text().toDouble();
        sharedData->control_variable_double[6] = ui->LE_COMCON_kd_yaw->text().toDouble();
    }
    else {
        for(int i=1; i<13; i++) sharedData->control_variable_double[i] = 0;
    }

    sharedData->flyingtrot_variable_double[0] = ui->LE_FLYINGTROT_GCOMP_M->text().toDouble();
    sharedData->flyingtrot_variable_double[1] = ui->LE_FLYINGTROT_GCOMP_S->text().toDouble();
}

void MainWindow::on_BTN_FLYINGTROT_READY_clicked()
{
    on_BTN_FLYINGTROT_SET_clicked();

    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = MOTION_START;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0]= Flyingtrot;
    sharedData->COMMAND = cmd.COMMAND_DATA;
    sharedData->NEWCOMMAND = true;

    if(ui->CB_LEGCON_HR->isChecked()) sharedData->task_legcon[HR] = true;
    else sharedData->task_legcon[HR] = false;
    if(ui->CB_LEGCON_HL->isChecked()) sharedData->task_legcon[HL] = true;
    else sharedData->task_legcon[HL] = false;
    if(ui->CB_LEGCON_FR->isChecked()) sharedData->task_legcon[FR] = true;
    else sharedData->task_legcon[FR] = false;
    if(ui->CB_LEGCON_FL->isChecked()) sharedData->task_legcon[FL] = true;
    else sharedData->task_legcon[FL] = false;

    sharedData->flyingtrot_variable_int[0] = 0;

    sharedData->flyingtrot_variable_double[0] = ui->LE_FLYINGTROT_GCOMP_M->text().toDouble();
    sharedData->flyingtrot_variable_double[1] = ui->LE_FLYINGTROT_GCOMP_S->text().toDouble();

    BasicSetup();
    LegControlSetup();
}

void MainWindow::on_BTN_FLYINGTROT_START_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0]= Flyingtrot;
    sharedData->COMMAND = cmd.COMMAND_DATA;
    sharedData->NEWCOMMAND = true;

    if(ui->CB_LEGCON_HR->isChecked()) sharedData->task_legcon[HR] = true;
    else sharedData->task_legcon[HR] = false;
    if(ui->CB_LEGCON_HL->isChecked()) sharedData->task_legcon[HL] = true;
    else sharedData->task_legcon[HL] = false;
    if(ui->CB_LEGCON_FR->isChecked()) sharedData->task_legcon[FR] = true;
    else sharedData->task_legcon[FR] = false;
    if(ui->CB_LEGCON_FL->isChecked()) sharedData->task_legcon[FL] = true;
    else sharedData->task_legcon[FL] = false;

    sharedData->flyingtrot_variable_int[0] = 1;

    sharedData->flyingtrot_variable_double[0] = ui->LE_FLYINGTROT_GCOMP_M->text().toDouble();
    sharedData->flyingtrot_variable_double[1] = ui->LE_FLYINGTROT_GCOMP_S->text().toDouble();

    BasicSetup();
}

void MainWindow::on_BTN_FLYINGTROT_STOP_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0]= Flyingtrot;
    sharedData->COMMAND = cmd.COMMAND_DATA;
    sharedData->NEWCOMMAND = true;

    sharedData->flyingtrot_variable_int[0] = 2;
}
