#ifndef MAIN_CONTROLLER_H
#define MAIN_CONTROLLER_H

#include "index_notation.h"
#include "JointInformation.h"
#include "RBCAN.h"
#include "hubodog5_struct.h"

using namespace std;

void main_controller();
void main_init();
void *main_nonRT(void *arg);
void set_ref_data();
void get_sensor_data();
void thread_MPC();

#endif // MAIN_CONTROLLER_H
