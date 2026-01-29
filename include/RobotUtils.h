#ifndef ROBOT_UTILS_H
#define ROBOT_UTILS_H

#include <string>

#include "AsyncLogger.h"
#include "nrcAPI.h"

struct SensorData {
    double fx, fy, fz;
    double mx, my, mz;
};

struct MyRobotState {
    double x, y, z, rz;
    double theta2, theta4;
};

bool setup_realtime();
void SystemStartup();
bool init_force_sensor_mapping();
void zero_force_sensor(AsyncLogger& logger);

SensorData read_force_sensor_da_raw();
SensorData read_force_sensor_xiao_raw();
SensorData read_force_sensor_da();
SensorData read_force_sensor_xiao();

MyRobotState read_robot_full_state();
bool perform_ik(NRC_Position& ref_acs, double x_m, double y_m, double z_m, double rz_rad, NRC_Position& res);

std::string MakeLogFileName();

#endif
