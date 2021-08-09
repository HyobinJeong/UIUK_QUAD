//
// Created by legendarypyunny on 2020-07-27.
//

#ifndef ROBOT_PARAMETERS
#define ROBOT_PARAMETERS

#define PI 3.14159265359
#define DEG2RAD(x) (x * PI / 180.0)
#define RAD2DEG(x) (x * 180.0 / PI)

//#define LAIKAGO
#define HUBODOG

extern const double gravity;
extern const double mu;
extern const double min_friction;
extern const double max_friction;
extern const double max_linear_acc;

extern const int num_states;
extern const int num_mpc_state;
extern const int num_inputs;
extern const int num_problem_unit;
extern const int num_legs;

#ifdef LAIKAGO
const double mass = 13.733;
const double hip_mass = 1.096;
const double thigh_mass = 1.528;
const double calf_mass = 0.241;

const double hip_com[] = {-0.001568, 0.008134, 0.000864};
const double thigh_com[] = {-0.000482, 0.02001, -0.031996};
const double calf_com[] = {0.002196, 0.000381, -0.12338};

const double Ixx = 0.073348887;
const double Iyy = 0.250684593;
const double Izz = 0.254469458;
const double hip_Ixx = 0.000822113;
const double hip_Iyy = 0.000983196;
const double hip_Izz = 0.000864753;
const double hip_Ixy = -4.982e-06;
const double hip_Iyz = 2.811e-06;
const double hip_Izx = -3.672e-05;

const double thigh_Ixx = 0.00991611;
const double thigh_Iyy = 0.009280083;
const double thigh_Izz = 0.00178256;
const double thigh_Ixy = 1.0388e-05;
const double thigh_Iyz = 8.511e-05;
const double thigh_Izx = 0.000250428;

const double calf_Ixx = 0.006181961;
const double calf_Iyy = 0.006196546;
const double calf_Izz = 3.4774e-05;
const double calf_Ixy = 2.37e-07;
const double calf_Iyz = 5.138e-06;
const double calf_Izx = -2.985e-06;

const double bd2imu[] = {-0.21935, 0, 0};
const double bd2hip[] = {0.21935, 0.0875, 0};
const double hip2pitch = 0.037;
const double foot_radius = 0.0265;
const double thigh_length = 0.25;
const double calf_length = 0.25;
const double init_angle = DEG2RAD(35);

#elif defined(HUBODOG)
extern const double mass;
extern const double hip_mass;
extern const double thigh_mass;
extern const double calf_mass;

extern const double hip_com[];
extern const double thigh_com[];
extern const double calf_com[];

extern const double Ixx;
extern const double Iyy;
extern const double Izz;
extern const double hip_Ixx;
extern const double hip_Iyy;
extern const double hip_Izz;
extern const double hip_Ixy;
extern const double hip_Iyz;
extern const double hip_Izx;

extern const double thigh_Ixx;
extern const double thigh_Iyy;
extern const double thigh_Izz;
extern const double thigh_Ixy;
extern const double thigh_Iyz;
extern const double thigh_Izx;

extern const double calf_Ixx;
extern const double calf_Iyy;
extern const double calf_Izz;
extern const double calf_Ixy;
extern const double calf_Iyz;
extern const double calf_Izx;

extern const double bd2imu[];
extern const double bd2hip[];
extern const double hip2pitch;
extern const double foot_radius;
extern const double thigh_length;
extern const double calf_length;
extern const double init_angle;

#elif ANYMAL

#endif

extern const double height;//foot_radius + (thigh_length + calf_length) * cos(init_angle);
extern const double shoulder_width;
extern const double shoulder_depth;

#endif




