#ifndef DAEMON_H
#define DAEMON_H

#include <unistd.h>
#include <iostream>
#include <alchemy/task.h>

#include "RBCAN.h"
#include "RBMotorController.h"
#include "RBIMUSensor.h"
#include "RBLog.h"
#include <QTimer>

//#define SIMULATION

#ifdef SIMULATION
#include "raisim/World.hpp"
#include "raisim/RaisimServer.hpp"
#endif

void    RBCore_SMInitialize();
int     RBCore_DBInitialize();
int     RBCore_CANInitialize();
int     RBCore_ThreadInitialize();

void    RBCore_RTThreadCon(void *);
void    RBCore_RTThreadCon2(void *);

void * RBCore_NRTThread(void *msg);
void * RBCore_NRTThread_Collision(void *msg);
void * RBCore_NRTThread_Beep(void *msg);

#endif // DAEMON_H
