#define _GNU_SOURCE         // 必须置于顶部，用于启用 Linux 实时特性
#include <iostream>
#include <vector>
#include <cmath>
#include <thread> 
#include <chrono>
#include <cstring>
#include <csignal>
#include <atomic>
#include <sched.h>          // 实时调度
#include <sys/mman.h>       // 内存锁定
#include <time.h>           // 高精度定时器
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

// 自定义头文件
#include "Admittance.h" 
#include "nrcAPI.h"
#include "AsyncLogger.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// 全局变量
std::atomic<bool> g_running{true};
void handle_sigint(int) { g_running = false; }

AsyncLogger logger; // 异步日志

struct SensorData {
    double fx, fy, fz; 
    double mx, my, mz;
};

struct MyRobotState {
    double x, y, z, rz;
    double theta2, theta4; 
};

// 传感器全局指针与变量
static unsigned char* g_force_ptrsda[6] = {nullptr};
static bool g_is_sensor_ready = false;
static SensorData g_sensor_offset = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
std::array<double, 7> target_joints;

// --- 实时性配置 ---
bool setup_realtime() {
    // 锁定物理内存
    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
        perror("mlockall");
        return false;
    }
    // 设置 FIFO 实时调度策略
    struct sched_param param;
    param.sched_priority = 80; 
    if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
        perror("sched_setscheduler");
        return false;
    }
    return true;
}

// --- 系统初始化与传感器 ---
void SystemStartup() {
    std::cout << "Robot Controller Starting..." << std::endl;
    NRC_StartController();
    while (NRC_GetControlInitComplete() != 1) { NRC_Delayms(100); }
    NRC_ClearAllError();
}

bool init_force_sensor_mapping() {
    unsigned int slaveNum = 6;
    unsigned short index = 0x6030;
    for(int i=0; i<6; ++i) {
        g_force_ptrsda[i] = NRC_GetPDOAddrMap(slaveNum, index, i + 1);
        if(g_force_ptrsda[i] == nullptr) return false;
    }
    g_is_sensor_ready = true;
    return true;
}

void zero_force_sensor() {
    const int sample_count = 10;
    SensorData temp = {0,0,0,0,0,0};
    for (int i = 0; i < sample_count; ++i) {
        if (g_is_sensor_ready) {
            temp.fx += *reinterpret_cast<float*>(g_force_ptrsda[0]);
            temp.fy += *reinterpret_cast<float*>(g_force_ptrsda[1]);
            temp.fz += *reinterpret_cast<float*>(g_force_ptrsda[2]);
            temp.mx += *reinterpret_cast<float*>(g_force_ptrsda[3]);
            temp.my += *reinterpret_cast<float*>(g_force_ptrsda[4]);
            temp.mz += *reinterpret_cast<float*>(g_force_ptrsda[5]);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    g_sensor_offset.fx = temp.fx / sample_count;
    g_sensor_offset.fy = temp.fy / sample_count;
    g_sensor_offset.fz = temp.fz / sample_count;
    g_sensor_offset.mx = temp.mx / sample_count;
    g_sensor_offset.my = temp.my / sample_count;
    g_sensor_offset.mz = temp.mz / sample_count;
    logger.log("Sensor Cleared.\n");
}

SensorData read_force_sensor() {
    if (!g_is_sensor_ready) return {0,0,0,0,0,0};
    return { 
        *reinterpret_cast<float*>(g_force_ptrsda[0]) - g_sensor_offset.fx,
        *reinterpret_cast<float*>(g_force_ptrsda[1]) - g_sensor_offset.fy,
        *reinterpret_cast<float*>(g_force_ptrsda[2]) - g_sensor_offset.fz,
        *reinterpret_cast<float*>(g_force_ptrsda[3]) - g_sensor_offset.mx,
        *reinterpret_cast<float*>(g_force_ptrsda[4]) - g_sensor_offset.my,
        *reinterpret_cast<float*>(g_force_ptrsda[5]) - g_sensor_offset.mz 
    };
}

MyRobotState get_robot_state() {
    MyRobotState s = {0};
    NRC_Position mcs, acs;
    if (NRC_GetCurrentPos(NRC_COORD::NRC_MCS, mcs) == 0) {
        s.x = mcs.pos[0] / 1000.0; s.y = mcs.pos[1] / 1000.0; s.z = mcs.pos[2] / 1000.0;
        s.rz = mcs.pos[5];
    }
    if (NRC_GetCurrentPos(NRC_COORD::NRC_ACS, acs) == 0) {
        s.theta2 = acs.pos[0] * (M_PI / 180.0);
        s.theta4 = acs.pos[3] * (M_PI / 180.0);
    }
    return s;
}

// --- 主逻辑 ---
int main() {
    signal(SIGINT, handle_sigint);
    SystemStartup();

    if (!setup_realtime()) {
        std::cerr << "Realtime Error! Use sudo." << std::endl;
        return -1;
    }

    // 机器人轨迹限制
    std::vector<double> vMax = {60, 60, 60, 60, 20, 20, 20};
    std::vector<double> avMax = {1500, 1500, 1500, 2000, 2000, 2000, 2000};
    std::vector<double> jMax = {1500, 2000, 2000, 2000, 2000, 2000, 2000};
    NRC_RKG_Open(vMax, avMax, jMax);

    // 导纳控制器
    double dt = 0.001; 
    Admittance4::Vec4 M = {130.0, 130.0, 110.0, 5};
    Admittance4::Vec4 D = {3000.0, 2000.0, 5000.0, 120}; // 略微增加末端阻尼
    Admittance4::Vec4 K = {0.0, 0.0, 0.0, 0.0};
    Admittance4 controller(M, D, K, dt);

    if (!init_force_sensor_mapping()) return -1;
    zero_force_sensor();

    MyRobotState init_s = get_robot_state();
    double b_x = init_s.x, b_y = init_s.y, b_z = init_s.z;
    double b_t4 = init_s.theta4, b_rz = init_s.rz;
    controller.set_state({0,0,0,0}, {0,0,0,0});

    // 定时器初始化
    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);
    long interval = 1000000; // 1ms

    bool last_f_sw = false, last_s_rd = false;

    while (g_running) {
        // 精准延时
        next.tv_nsec += interval;
        while (next.tv_nsec >= 1000000000L) { next.tv_nsec -= 1000000000L; next.tv_sec++; }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);

        // 使能逻辑
        bool s_rd = (NRC_ReadBoolVar(4) == 1);
        if(s_rd && !last_s_rd) { NRC_SetServoReadyStatus(1); NRC_PowerOn(); }
        if(!s_rd && last_s_rd) { NRC_SetServoReadyStatus(0); NRC_PowerOff(); }
        last_s_rd = s_rd;

        // 力控逻辑
        bool f_sw = (NRC_ReadBoolVar(1) == 1);
        if (f_sw && !last_f_sw) {
            zero_force_sensor();
            init_s = get_robot_state();
            b_x = init_s.x; b_y = init_s.y; b_z = init_s.z;
            b_t4 = init_s.theta4; b_rz = init_s.rz;
            controller.set_state({0,0,0,0}, {0,0,0,0});
            logger.log("Force Control Active.\n");
        }
        if (!f_sw && last_f_sw) { NRC_RKG_Stop(); NRC_PowerOff(); }
        last_f_sw = f_sw;

        if (!f_sw) continue;

        // 算法计算
        MyRobotState curr = get_robot_state();
        SensorData ft = read_force_sensor();

        // 滤波死区
        if (std::abs(ft.fx) < 2.0) ft.fx = 0.0;
        if (std::abs(ft.fy) < 2.0) ft.fy = 0.0;
        if (std::abs(ft.fz) < 2.5) ft.fz = 0.0;
        if (std::abs(ft.mz) < 0.5) ft.mz = 0.0;

        auto res = controller.update({ft.fx, ft.fy, ft.fz, ft.mz});
        Admittance4::Vec4 target = res.first;

        // 坐标投影
        double ang = -curr.rz;
        double dx = target[0] * std::cos(ang) - target[1] * std::sin(ang);
        double dy = target[0] * std::sin(ang) + target[1] * std::cos(ang);
        
        NRC_Position ref_acs, ik_res;
        NRC_GetCurrentPos(NRC_COORD::NRC_ACS, ref_acs);
        
        NRC_Position mcs_target(NRC_COORD::NRC_MCS, (b_x+dx)*1000, (b_y+dy)*1000, (b_z+target[2])*1000, 3.14159, 0, b_rz);
        
        if (NRC_MCStoACS(ref_acs, mcs_target, ik_res) == 0) {
            target_joints[0] = ik_res.pos[0];
            target_joints[1] = ik_res.pos[1];
            target_joints[2] = ik_res.pos[2];
            target_joints[3] = (b_t4 + target[3]) * 180.0 / M_PI;
            NRC_Set_ServoJ_Pos(target_joints);
        }
    }

    NRC_RKG_Stop();
    NRC_PowerOff();
    return 0;
}