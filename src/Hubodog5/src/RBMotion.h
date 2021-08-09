#ifndef RBMOTION_H
#define RBMOTION_H

#include "JointInformation.h"
#include "RBCAN.h"

using namespace std;

int RBCore_DBInitialize();
int SYS_SET_THREAD_AFFINITY(void* (*t_handler)(void *), int t_cpu_no, const char* t_name);


#endif // RBMOTION_H
