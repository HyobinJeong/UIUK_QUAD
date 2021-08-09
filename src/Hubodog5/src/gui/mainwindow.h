#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTableWidget>
#include <QTimer>

#include "ModelDialog.h"



#include "RBSharedMemory.h"
#include "JointInformation.h"
#include "RBMotorController.h"
#include "RBIMUSensor.h"
//#include "huboquaddialog.h"
//#include "mingyudialog.h"
//#include "junny_estimation_dialog.h"

#include "qcustomplot.h"
#include "RBJoystick.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();


    ModelDialog     *model;
//    HuboQuadDialog  *huboquad;
//    Dialog    *mgdialog;
//    Junny_Estimation_Dialog *junny_estimation_dialog;

private slots:
    // graph part
    void GraphInitialize();
    void DisplayUpdate_Main();
    void DispalyUpdate_BasicTest();
    void DispalyUpdate_MPC();

    void selectionChanged(QCustomPlot* &ui_test);
    void mouseWheel(QCustomPlot* &ui_test);
    void mousePress(QCustomPlot* &ui_test);


    void BasicSetup();
    void LegControlSetup();
    void MPCGainUpdate();

    void on_TW_0_itemSelectionChanged();

    void on_BTN_REF_ON_clicked();

    void on_BTN_REF_OFF_clicked();

    void on_BTN_CAN_CHECK_clicked();

    void on_BTN_FIND_HOME_clicked();

    void on_BTN_RESET_PROCESS_1_clicked();

    void on_BTN_RESET_PROCESS_2_clicked();

    void on_BTN_RESET_PROCESS_3_clicked();

    void on_BTN_JOY_CONNECT_clicked();

    void on_BTN_JOY_DISCONNECT_clicked();

    void on_BTN_IMU_ENABLE_clicked();

    void on_BTN_IMU_NULL_clicked();

    void on_BTN_IMU_RESET_clicked();

    void on_BTN_READYPOS_clicked();

    void on_BTN_MOUNTPOS_clicked();

    void on_BTN_OFF_clicked();

    void on_BTN_DEMO_clicked();

    void on_BTN_OneLegTest_clicked();

    void on_BTN_OneLegTest_ON_clicked();

    void on_BTN_OneLegTest_OFF_clicked();

    void on_BTN_STOP_clicked();

    void on_BTN_MG_timerstop_clicked();

    void on_BTN_DANCE01_clicked();

    void on_BTN_RESET_PROCESS_4_clicked();

    void on_BTN_MG_Trot_clicked();

    void on_BTN_MG_stepping_clicked();

    void on_BTN_MPCgainset_clicked();

    void on_BTN_MG_timerstop_2_clicked();

    void on_BTN_ENC_CHECK_clicked();

    void on_BTN_JOG_ON_clicked();

    void on_BTN_JOG_SPEED_1_clicked();

    void on_BTN_JOG_SPEED_2_clicked();

    void on_BTN_JOG_SPEED_3_clicked();

    void on_BTN_JOG_SPEED_4_clicked();

    void on_BTN_JOG_MODE_1_clicked();

    void on_BTN_JOG_MODE_2_clicked();

    void on_BTN_JOG_UP_pressed();

    void on_BTN_JOG_UP_released();

    void on_BTN_JOG_DOWN_pressed();

    void on_BTN_JOG_DOWN_released();

    void on_BTN_JOG_UP_clicked();

    void on_BTN_JOG_DOWN_clicked();

    void on_BTN_TROTTEST_START_clicked();

    void on_BTN_FOC_GAINSET_clicked();

    void on_BTN_TROTTEST_STOP_clicked();

    void on_BTN_TROTTEST_READY_clicked();

    void on_BTN_TROTTEST_ONESTEP_clicked();

    void on_BTN_RESET_PROCESS_5_clicked();

    void on_BTN_RESET_PROCESS_6_clicked();

    void on_BTN_IMU_RESET0_clicked();

    void on_BTN_IMU_SET_clicked();

    void on_BTN_TROTTEST_SET_clicked();

    void on_BTN_FOC_GAINREQ_clicked();

    void on_BTN_POS_GAINSET_clicked();

    void on_BTN_POS_GAINREQ_clicked();

    void on_BTN_HOPTEST_SET_clicked();

    void on_BTN_HOPTEST_READY_clicked();

    void on_BTN_HOPTEST_START_clicked();

    void on_BTN_HOPTEST_STOP_clicked();

    void on_BTN_CONTROLLER_SET_clicked();

    void on_BTN_INCGAIN_clicked();

    void on_BTN_GROUNDPOS_clicked();

    void on_BTN_GREADYPOS_clicked();

    void on_BTN_HOMMINGPOS_clicked();

    void on_BTN_BACKFLIP_clicked();

    void on_BTN_JUMP_clicked();

    void on_BTN_SIM_RESET_clicked();

    void on_BTN_IMU_ALPHA_clicked();

    void on_BTN_IMU_ALPHA_2_clicked();

    void on_BTN_BACKFLIP_H_clicked();

    void on_BTN_FALLPOS_clicked();

    void on_BTN_FALLRECOVER_clicked();

    void on_BTN_FLYINGTROT_SET_clicked();

    void on_BTN_FLYINGTROT_READY_clicked();

    void on_BTN_FLYINGTROT_START_clicked();

    void on_BTN_FLYINGTROT_STOP_clicked();

private:
    Ui::MainWindow *ui;
    QTimer      *displayTimer;
    QTimer		*graphTimer;
    RBJoystick      *joy;
    int         JoyCommand;
    char        oldB[12];


    void    InitTable(QTableWidget *table, const std::string j_names[], int num);
    void    ChangeSelectedJoint();
    int     select_working;
    int     lastSelected;

    int     err_clr_start;
    int     err_clr_cnt;
    int     err_clr_jnt_id;
    int     err_clr_jnt_ch;


};

#endif // MAINWINDOW_H
