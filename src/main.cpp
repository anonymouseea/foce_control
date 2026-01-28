#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include <iostream>
#include <vector>
#include <cmath>
#include <thread> 
#include <chrono>
#include <csignal>
#include <atomic>
#include <sched.h>          // 实时调度
#include <sys/mman.h>       // 内存锁定
#include <time.h>           // 高精度时钟
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <fstream>
#include <ctime>
#include <sstream>
#include <iomanip>

#include "Admittance.h" 
#include "nrcAPI.h"
#include "AsyncLogger.h"

#include <sys/stat.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

std::atomic<bool> g_running{true};
void handle_sigint(int) { g_running = false; }

AsyncLogger logger;

struct SensorData {
    double fx, fy, fz; 
    double mx, my, mz; 
};

struct MyRobotState {
    double x, y, z, rz; 
    double theta2, theta4; 
};

static unsigned char* g_force_ptrsda[6] = {nullptr};
static unsigned char* g_force_ptrsxiao[6] = {nullptr};
static bool g_is_sensor_ready = false;
static SensorData g_sensor_offset = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
static SensorData g_sensor_offset_xiao = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
std::array<double, 7> target_joints;

SensorData ft;
double dead_zone_f, dead_zone_m; 

bool setup_realtime() {
  
    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
        perror("mlockall failed");
        return false;
    }
    
    struct sched_param param;
    param.sched_priority = 80; 
    if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
        perror("sched_setscheduler failed (Try sudo)");
        return false;
    }
    return true;
}

// 系统初始化
void SystemStartup() {
  std::cout << "库版本：" << NRC_GetNexMotionLibVersion() << std::endl;
  //启动控制系统
  NRC_StartController();
  //检测控制系统是否初始化完成
  while (NRC_GetControlInitComplete() != 1 ) {
    NRC_Delayms(100);   //延时100ms
  }
  //清除所有错误
  NRC_ClearAllError();
  std::cout << "----" << NRC_GetControlInitComplete() << std::endl;
  std::cout << "StartController Success" << std::endl;
  std::cout << "获取同步版本号" << NRC_GetSyncVersion() << std::endl;
  NRC_Delayms(200);
}

bool init_force_sensor_mapping() {
    unsigned short index = 0x6030;
    bool allSuccess = true;
    for(int i=0; i<6; ++i) {
        g_force_ptrsda[i] = NRC_GetPDOAddrMap(6, index, i + 1); // 大量程 Slave 6
        g_force_ptrsxiao[i] = NRC_GetPDOAddrMap(7, index, i + 1); // 小量程 Slave 7
        if(!g_force_ptrsda[i] || !g_force_ptrsxiao[i]) allSuccess = false;
    }
    if (allSuccess) g_is_sensor_ready = true;
    return allSuccess;  
}

void zero_force_sensor() {
    logger.log("开始传感器清零 (采样中，请勿触摸机械臂)...\n");
    
    for (int i = 0; i < 10; ++i) {
        if (g_is_sensor_ready) {
            float dummy = *reinterpret_cast<float*>(g_force_ptrsda[0]);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    const int sample_count = 50; 
    SensorData sum_da = {0, 0, 0, 0, 0, 0};
    SensorData sum_xiao = {0, 0, 0, 0, 0, 0};

    for (int i = 0; i < sample_count; ++i) {
        if (g_is_sensor_ready) {
            // 大量程传感器累加
            sum_da.fx += (double)*reinterpret_cast<float*>(g_force_ptrsda[0]);
            sum_da.fy += (double)*reinterpret_cast<float*>(g_force_ptrsda[1]);
            sum_da.fz += (double)*reinterpret_cast<float*>(g_force_ptrsda[2]);
            sum_da.mx += (double)*reinterpret_cast<float*>(g_force_ptrsda[3]);
            sum_da.my += (double)*reinterpret_cast<float*>(g_force_ptrsda[4]);
            sum_da.mz += (double)*reinterpret_cast<float*>(g_force_ptrsda[5]);

            // 小量程传感器累加
            sum_xiao.fx += (double)*reinterpret_cast<float*>(g_force_ptrsxiao[0]);
            sum_xiao.fy += (double)*reinterpret_cast<float*>(g_force_ptrsxiao[1]);
            sum_xiao.fz += (double)*reinterpret_cast<float*>(g_force_ptrsxiao[2]);
            sum_xiao.mx += (double)*reinterpret_cast<float*>(g_force_ptrsxiao[3]);
            sum_xiao.my += (double)*reinterpret_cast<float*>(g_force_ptrsxiao[4]);
            sum_xiao.mz += (double)*reinterpret_cast<float*>(g_force_ptrsxiao[5]);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2)); // 每 2ms 采一个点
    }

    // --- 大量程偏移量计算
    g_sensor_offset.fx = sum_da.fx / sample_count;
    g_sensor_offset.fy = sum_da.fy / sample_count;
    g_sensor_offset.fz = sum_da.fz / sample_count;
    g_sensor_offset.mx = sum_da.mx / sample_count;
    g_sensor_offset.my = sum_da.my / sample_count;
    g_sensor_offset.mz = sum_da.mz / sample_count;

    // --- 小量程偏移量计算 
    g_sensor_offset_xiao.fx = sum_xiao.fx / sample_count;
    g_sensor_offset_xiao.fy = sum_xiao.fy / sample_count;
    g_sensor_offset_xiao.fz = sum_xiao.fz / sample_count;
    g_sensor_offset_xiao.mx = sum_xiao.mx / sample_count;
    g_sensor_offset_xiao.my = sum_xiao.my / sample_count;
    g_sensor_offset_xiao.mz = sum_xiao.mz / sample_count;

    logger.log("传感器清零完成。" );
}

// 原始数据获取函数
SensorData read_force_sensor_da_raw() {
    if (!g_is_sensor_ready) return {0,0,0,0,0,0};
    return {(double)*reinterpret_cast<float*>(g_force_ptrsda[0]), (double)*reinterpret_cast<float*>(g_force_ptrsda[1]), (double)*reinterpret_cast<float*>(g_force_ptrsda[2]),
            (double)*reinterpret_cast<float*>(g_force_ptrsda[3]), (double)*reinterpret_cast<float*>(g_force_ptrsda[4]), (double)*reinterpret_cast<float*>(g_force_ptrsda[5])};
}
SensorData read_force_sensor_xiao_raw() {
    if (!g_is_sensor_ready) return {0,0,0,0,0,0};
    return {(double)*reinterpret_cast<float*>(g_force_ptrsxiao[0]), (double)*reinterpret_cast<float*>(g_force_ptrsxiao[1]), (double)*reinterpret_cast<float*>(g_force_ptrsxiao[2]),
            (double)*reinterpret_cast<float*>(g_force_ptrsxiao[3]), (double)*reinterpret_cast<float*>(g_force_ptrsxiao[4]), (double)*reinterpret_cast<float*>(g_force_ptrsxiao[5])};
}


// 减去偏移后的数据
SensorData read_force_sensor_da() {
    SensorData r = read_force_sensor_da_raw();
    return {r.fx - g_sensor_offset.fx, r.fy - g_sensor_offset.fy, r.fz - g_sensor_offset.fz, r.mx - g_sensor_offset.mx, r.my - g_sensor_offset.my, r.mz - g_sensor_offset.mz};
}
SensorData read_force_sensor_xiao() {
    SensorData r = read_force_sensor_xiao_raw();
    return {r.fx - g_sensor_offset_xiao.fx, r.fy - g_sensor_offset_xiao.fy, r.fz - g_sensor_offset_xiao.fz, r.mx - g_sensor_offset_xiao.mx, r.my - g_sensor_offset_xiao.my, r.mz - g_sensor_offset_xiao.mz};
}

MyRobotState read_robot_full_state() {
    MyRobotState state = {0};
    NRC_Position mcsPos, acsPos;
    if (NRC_GetCurrentPos(NRC_COORD::NRC_MCS, mcsPos) == 0) {
        state.x = mcsPos.pos[0]/1000.0; state.y = mcsPos.pos[1]/1000.0; state.z = mcsPos.pos[2]/1000.0; state.rz = mcsPos.pos[5];
    }
    if (NRC_GetCurrentPos(NRC_COORD::NRC_ACS, acsPos) == 0) {
        state.theta2 = acsPos.pos[0] * (M_PI / 180.0); state.theta4 = acsPos.pos[3] * (M_PI / 180.0);
    }
    return state;
}

bool perform_ik(NRC_Position& ref_acs, double x_m, double y_m, double z_m, double rz_rad, NRC_Position& res) {
    NRC_Position posMCS(NRC_COORD::NRC_MCS, x_m*1000.0, y_m*1000.0, z_m*1000.0, 3.14159, 0, rz_rad);
    return (NRC_MCStoACS(ref_acs, posMCS, res) == 0);
}

static std::string MakeLogFileName() {
    std::time_t t = std::time(nullptr);
    std::tm tm_val;
    localtime_r(&t, &tm_val);
    char buf[32] = {0};
    std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", &tm_val);
    std::ostringstream oss;
    oss << "szl_log/run_" << buf << ".log";
    return oss.str();
}

// 统一初始化入口
bool InitializeSystem() {
    logger.log("[INIT] System startup begin\n");
    SystemStartup();

    const std::string log_file = MakeLogFileName();
    // 确保日志目录存在（相对路径：程序当前工作目录下的 szl_log/）
    ::mkdir("szl_log", 0755);
    logger.setLogFile(log_file);
    logger.log(std::string("[LOG] file=") + log_file);

    signal(SIGINT, handle_sigint); 
    
    if (!init_force_sensor_mapping()) {
        logger.log("[INIT] init_force_sensor_mapping failed\n");
        return false;
    }

    zero_force_sensor();
    // 设置示教模式
    NRC_SetOperationMode(NRC_TEACH_);
    
    return true;
}


int main() 
{

    if (!InitializeSystem()) {
        logger.log("[INIT] InitializeSystem failed, exit\n");
        return -1;
    }

    Admittance4 controller( {130.0, 130.0, 110.0, 5},
                            {3000.0, 3000.0, 5000.0, 120},
                            {0,0,0,0}, 0.001);

    MyRobotState init_s = read_robot_full_state();
    double base_x = init_s.x, base_y = init_s.y, base_z = init_s.z, base_t4 = init_s.theta4, initial_total_rz = init_s.rz;

    enum ControlMode { MODE_POS, MODE_ROT };
    ControlMode current_mode = MODE_POS;
    Admittance4::Vec4 last_target_tool = {0,0,0,0}, active_pos_lock = {0,0,0,0};
    double locked_rz = 0.0;

   
    std::thread force_feedback_thread([](){
        struct sched_param param;
        param.sched_priority = 0;
        pthread_setschedparam(pthread_self(), SCHED_OTHER, &param);
        while (g_running) {
            SensorData data_xiao = read_force_sensor_xiao_raw();
            NRC_SetDoubleVar(7, data_xiao.fx); NRC_SetDoubleVar(8, data_xiao.fy); NRC_SetDoubleVar(9, data_xiao.fz);
            NRC_SetDoubleVar(10, data_xiao.mx); NRC_SetDoubleVar(11, data_xiao.my); NRC_SetDoubleVar(12, data_xiao.mz);
            
            SensorData data_da = read_force_sensor_da_raw();
            NRC_SetDoubleVar(13, data_da.fx); NRC_SetDoubleVar(14, data_da.fy); NRC_SetDoubleVar(15, data_da.fz);
            NRC_SetDoubleVar(16, data_da.mx); NRC_SetDoubleVar(17, data_da.my); NRC_SetDoubleVar(18, data_da.mz);
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
    });
    force_feedback_thread.detach();

    bool last_force_switch = false, last_zero_switch = false, last_remote_switch = false, last_servo_ready = false;

    // --- 高精度硬实时循环起始 ---
    struct timespec next_p;
    clock_gettime(CLOCK_MONOTONIC, &next_p);

    while (g_running) 
    {
        // 设置下一个唤醒时间点 (1ms 周期)
        next_p.tv_nsec += 1000000; 
        while (next_p.tv_nsec >= 1000000000L) { next_p.tv_nsec -= 1000000000L; next_p.tv_sec++; }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_p, NULL);

        // 传感器清零逻辑
        bool zero_on = (NRC_ReadBoolVar(2) == 1);
        if (zero_on && !last_zero_switch) { zero_force_sensor(); }
        last_zero_switch = zero_on;

        // 控开关逻辑 
        bool force_on = (NRC_ReadBoolVar(1) == 1);
       
        if (force_on && !last_force_switch) {
            
            logger.log("[CONTROL] Force control enabled\n");
            NRC_ClearAllError();
            zero_force_sensor();
            NRC_SetServoReadyStatus(1); 
            NRC_PowerOn();
            init_s = read_robot_full_state();
            base_x = init_s.x; base_y = init_s.y; base_z = init_s.z; base_t4 = init_s.theta4; initial_total_rz = init_s.rz;
            controller.set_state({0,0,0,0}, {0,0,0,0});
            last_target_tool = {0,0,0,0};
            NRC_RKG_Open({40,40,60,60,20,20,20},{1500,1500,1500,2000,2000,2000,2000},{2000,2000,2000,2000,2000,2000,2000});

        }
        //关闭力控
        if (!force_on && last_force_switch) {logger.log("[CONTROL] Force control disabled\n"); NRC_RKG_Stop(); NRC_PowerOff(); }
        last_force_switch = force_on;
        if (!force_on) continue;
        
        MyRobotState curr_s = read_robot_full_state();
        NRC_Position ref_acs; NRC_GetCurrentPos(NRC_COORD::NRC_ACS, ref_acs);

        SensorData ft_da = read_force_sensor_da();
        SensorData ft_xiao = read_force_sensor_xiao();

        // 传感器切换逻辑
        static bool last_use_small_sensor = false;
        bool use_small_sensor = (NRC_ReadBoolVar(5) == 1); 
        if (use_small_sensor != last_use_small_sensor) {
            // 切换传感器时，保持当前位置偏移，将速度置0，防止跳动
            controller.set_state(last_target_tool, {0,0,0,0});
            
            logger.log(std::string("[传感器切换] 使用") + (use_small_sensor ? "小量程" : "大量程") + "传感器\n");
            last_use_small_sensor = use_small_sensor; 
        }

        if (use_small_sensor) {
            ft = ft_xiao;
            // 小量程传感器的死区
            dead_zone_f = 1.0; 
            dead_zone_m = 0.2;
        } else {
            ft = ft_da;
            // 大量程传感器的死区
            dead_zone_f = 2.0;
            dead_zone_m = 0.5;
        }

        // 滤波与死区
        if (std::abs(ft.fx) < dead_zone_f) ft.fx = 0; if (std::abs(ft.fy) < dead_zone_f) ft.fy = 0;
        if (std::abs(ft.fz) < dead_zone_f) ft.fz = 0; if (std::abs(ft.mz) < dead_zone_m) ft.mz = 0;

        // 模式切换 (IO 1.1)
        ControlMode target_mode = (NRC_ReadDigInByBoard(1, 1) == 1) ? MODE_ROT : MODE_POS;
        if (target_mode != current_mode) {
            current_mode = target_mode;
            logger.log(std::string("[MODE] Switched to ") + (current_mode == MODE_ROT ? "ROT" : "POS") + " mode\n");
            controller.set_state(last_target_tool, {0,0,0,0}); 
            if (current_mode == MODE_ROT) { active_pos_lock = last_target_tool; locked_rz = curr_s.rz; }
        }
        
        if (current_mode == MODE_POS) ft.mz = 0;

        auto result = controller.update({ft.fx, ft.fy, ft.fz, ft.mz});
        Admittance4::Vec4 target_tool = result.first;

        if (current_mode == MODE_ROT) { target_tool[0] = active_pos_lock[0]; target_tool[1] = active_pos_lock[1]; target_tool[2] = active_pos_lock[2]; }
        last_target_tool = target_tool;

        // 工具系转基座系投影
        double angle = (current_mode == MODE_ROT) ? -locked_rz : -curr_s.rz;
        double dx = target_tool[0] * cos(angle) - target_tool[1] * sin(angle);
        double dy = target_tool[0] * sin(angle) + target_tool[1] * cos(angle);

        NRC_Position ik_res;
        if (perform_ik(ref_acs, base_x + dx, base_y + dy, base_z + target_tool[2], initial_total_rz, ik_res)) {
            target_joints = {ik_res.pos[0], ik_res.pos[1], ik_res.pos[2], (base_t4 + target_tool[3]) * 180.0/M_PI, 0, 0, 0};
            NRC_Set_ServoJ_Pos(target_joints);
        }
    }

    return 0;
}