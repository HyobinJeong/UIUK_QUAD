#ifndef JOINT_INFORMATION_H
#define JOINT_INFORMATION_H

#include <iostream>

#define MOVE_RELATIVE   1
#define MOVE_ABSOLUTE   0

#define PI			3.141592653589793
#define D2R			1.745329251994330e-2
#define R2D			5.729577951308232e1


// Utility ----------------------------
#define LOG_FAIL    "\033[1;31m"
#define LOG_GOOD    "\033[1;32m"
#define LOG_WARN    "\033[1;33m"
#define LOG_NORMAL  "\033[0m"


enum JointSequentialNumber
{
    J0 = 0,
    J1,
    J2,
    J3,
    J4,
    J5,
    J6,
    J7,
    J8,
    J9,
    J10,
    J11,
    NO_OF_JOINTSJ
};


const std::string JointNameList[NO_OF_JOINTSJ] = {
    "JNT0",
    "JNT1",
    "JNT2",
    "JNT3",
    "JNT4",
    "JNT5",
    "JNT6",
    "JNT7",
    "JNT8",
    "JNT9",
    "JNT10",
    "JNT11"
};

const struct {
    int id;
    int ch;
} MC_ID_CH_Pairs[NO_OF_JOINTSJ] = {
    {0, 0}, // JNT0
    {1, 0}, // JNT1
    {2, 0}, // JNT2
    {3, 0}, // JNT3
    {4, 0}, // JNT4
    {5, 0}, // JNT5
    {6, 0}, // JNT6
    {7, 0}, // JNT7
    {8, 0}, // JNT8
    {9, 0}, // JNT9
    {10, 0}, // JNT10
    {11, 0}, // JNT11
};

inline int MC_GetID(int jnum){
    return MC_ID_CH_Pairs[jnum].id;
}
inline int MC_GetCH(int jnum){
    return MC_ID_CH_Pairs[jnum].ch;
}

#endif // JOINT_INFORMATION_H
