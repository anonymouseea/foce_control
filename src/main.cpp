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
#include <sched.h>          
#include <sys/mman.h>       
#include <time.h>           
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
#include "RobotUtils.h"
#include "ControlLoop.h"

#include <sys/stat.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

std::atomic<bool> g_running{true};
void handle_sigint(int) { g_running = false; }

AsyncLogger logger;

// 统一初始化入口
bool InitializeSystem() {
    logger.log("[INIT] 系统启动\n");

    // 智能启动
    if (NRC_GetControlInitComplete() != 1) {
        logger.log("[INIT] 检测到系统未初始化，执行SystemStartup()...\n");
        SystemStartup();
    } else {
        logger.log("[INIT] 检测到系统已在运行，跳过启动步骤。\n");
    }
    NRC_ClearAllError();

    // 日志
    const std::string log_file = MakeLogFileName();
    ::mkdir("szl_log", 0777); // 改成 777 防以后权限麻烦
    logger.setLogFile(log_file);
    logger.log(std::string("[LOG] file=") + log_file);

    signal(SIGINT, handle_sigint); 
    
    if (!init_force_sensor_mapping()) {
        logger.log("[INIT] 初始化传感器地址失败\n");
        return false;
    }
    
    // zero_force_sensor(logger); // 删掉或注释：反正 ControlLoop 里开力控时会清
    // NRC_SetOperationMode(NRC_TEACH_); // 删掉：防止自启动冲突
    
    return true;
}


int main() 
{
    if (!InitializeSystem()) {logger.log("[INIT] 初始化系统失败，退出\n");return -1;}
    RunControlLoop(logger, g_running);
    return 0;
}