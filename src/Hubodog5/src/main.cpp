#include "mainwindow.h"
#include <QApplication>

#include "RBDaemon.h"
#include "RBLANComm.h"

int     _IS_WORKING;
int     _NO_OF_MC;
int     _IS_NEW_CAN_DATA;
int     _IS_CAN_OK;


RBLANComm   *lanHandler;
RBCAN       *canHandler;

#ifdef SIMULATION
extern raisim::ArticulatedSystem *robot;
extern raisim::RaisimServer *server;
#endif

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    w.show();
    w.move(100,50);


    _IS_WORKING = true;

    canHandler = new RBCAN();
    usleep(200*1000);
    lanHandler = new RBLANComm();

#ifdef SIMULATION
  auto binaryPath = raisim::Path::setFromArgv(argv[0]);
  raisim::World world;
  world.setTimeStep(RT_TIMER_PERIOD_MS / 1000);
  world.addGround();

//  raisim::Cylinder *cyl;
//  cyl = world.addCylinder(0.1, 1, 1000);
//  cyl->setPosition(0, 0, 0.5);

  robot = world.addArticulatedSystem(binaryPath.getDirectory() + R"(\..\src\RBModel\hubodog\hubodog.urdf)");
  Eigen::VectorXd joint_pos(robot->getGeneralizedCoordinateDim()), joint_vel(robot->getDOF());
  joint_pos.setZero();
  joint_vel.setZero();
  for (int i = 0; i < 4; i++)
    joint_pos.segment(7 + 3*i, 3) = Eigen::Vector3d(0, 30*D2R, -2 * 30*D2R);
  joint_pos(3) = 1;
  joint_pos(2) = 0.45;
//  joint_pos(2) = 1;
  robot->setState(joint_pos, joint_vel);

  Eigen::VectorXd jointPgain(robot->getDOF()), jointDgain(robot->getDOF());
  jointPgain.tail(12).setConstant(10000);
  jointDgain.tail(12).setConstant(10);
  robot->setPdGains(jointPgain, jointDgain);

  robot->setGeneralizedCoordinate(joint_pos);
  robot->setGeneralizedForce(Eigen::VectorXd::Zero(robot->getDOF()));
  robot->setPdTarget(joint_pos, joint_vel);

  server = new raisim::RaisimServer(&world);
  server->launchServer();
  server->focusOn(robot);
#endif

    RBCore_SMInitialize();
    RBCore_DBInitialize();
//    RBCore_CANInitialize();
    RBCore_ThreadInitialize();

    return a.exec();
}
