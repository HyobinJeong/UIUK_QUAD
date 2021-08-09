#include "robot_parameters.h"

const double gravity = 9.81;
const double mu = 0.6;
const double min_friction = 10;
const double max_friction = 400;
const double max_linear_acc = 6;

const int num_states = 18;
const int num_mpc_state = 12;
const int num_inputs = 12;
const int num_problem_unit = num_mpc_state + num_inputs;
const int num_legs = 4;

const double mass = 11.5756;
const double hip_mass = 2.38674;
const double thigh_mass = 0.67231;
const double calf_mass = 0.173;

const double hip_com[] = {-4.51e-03, 17.58e-03, 0.04e-03};
const double thigh_com[] = {-2.26e-03, 7.68e-03, -41.12e-03};
const double calf_com[] = {0.75e-03, 0.35e-03, -88.03e-03};

const double Ixx = 66750053.32e-09;
const double Iyy = 124905646.11e-09;
const double Izz = 177312065.71e-09;
const double hip_Ixx = 3437636.42e-09;
const double hip_Iyy = 2718636.62e-09;
const double hip_Izz = 3437712.56e-09;
const double hip_Ixy = -120456.91e-09;
const double hip_Iyz = 3826.95e-09;
const double hip_Izx = 569.13e-09;

const double thigh_Ixx = 3146944.77e-09;
const double thigh_Iyy = 3429086.75e-09;
const double thigh_Izz = 636166.40e-09;
const double thigh_Ixy = 4356.11e-09;
const double thigh_Iyz = 158744.35e-09;
const double thigh_Izx = 98512.25e-09;

const double calf_Ixx = 1256859.90e-09;
const double calf_Iyy = 1263569.86e-09;
const double calf_Izz = 18848.54e-09;
const double calf_Ixy = 13.94e-09;
const double calf_Iyz = -10047.10e-09;
const double calf_Izx = -1349.76e-09;

const double bd2imu[] = {-0.08, -0.05, -0.02};
const double bd2hip[] = {0.22445, 0.07946, 0};
const double hip2pitch = 0.06995;
const double foot_radius = 0.025;
const double thigh_length = 0.225;
const double calf_length = 0.225;
const double init_angle = DEG2RAD(30);

const double height = 0.28;//foot_radius + (thigh_length + calf_length) * cos(init_angle);
const double shoulder_width = 2 * (bd2hip[1] + hip2pitch);
const double shoulder_depth = 2 * bd2hip[0];
