#include <iostream>
#include <vector>
#include <cmath>
#include <thread> 
#include <chrono>
#include <cstring>
#include <csignal>
#include <atomic>
#include "Admittance.h" 
#include "nrcAPI.h"
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include "AsyncLogger.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

AsyncLogger logger;

struct SensorData {
    double fx, fy, fz; // 工具系下的力
    double mx, my, mz; // 力矩
};

struct MyRobotState {
    double x, y, z, rz; // MCS: m, m, m, rad
    double theta2, theta4; // ACS: rad
};

static unsigned char* g_force_ptrsda[6] = {nullptr};
static unsigned char* g_force_ptrsxiao[6] = {nullptr};
static bool g_is_sensor_ready = false;
static SensorData g_sensor_offset = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // <--- 1. 添加全局偏移量变量
static SensorData g_sensor_offset_xiao = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // 小量程传感器偏移量
std::array<double, 7> target_joints;


void SystemStartup() {
  //输出Nexmotion版本库信息
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






// 初始化传感器映射（大量程传感器)
bool init_force_sensor_mapping() {
    //大量程
    unsigned int slaveNum = 6;
    unsigned short index = 0x6030;
    bool allSuccess = true;
    for(int i=0; i<6; ++i) {
        g_force_ptrsda[i] = NRC_GetPDOAddrMap(slaveNum, index, i + 1);
        if(g_force_ptrsda[i] == nullptr) allSuccess = false;
    }
    //小量程
    unsigned int slaveNum1 = 7;
    unsigned short index1 = 0x6030;
    // bool allSuccess = true;
    for(int i=0; i<6; ++i) {
        g_force_ptrsxiao[i] = NRC_GetPDOAddrMap(slaveNum1, index1, i + 1);
        if(g_force_ptrsxiao[i] == nullptr) allSuccess = false;
    }
    if (allSuccess) { g_is_sensor_ready = true; return true; }
    return false;
}

// 2. 添加传感器清零函数
void zero_force_sensor() {
    std::cout << "正在执行传感器清零操作..." << std::endl;
    // 多次读取求平均，使零点更准确
    const int sample_count = 10;
    SensorData temp_offset = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};      // 大量程累积
    SensorData temp_offset_xiao = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // 小量程累积
    for (int i = 0; i < sample_count; ++i) {
        if (g_is_sensor_ready) {
            float da_fx = *reinterpret_cast<float*>(g_force_ptrsda[0]);
            float da_fy = *reinterpret_cast<float*>(g_force_ptrsda[1]);
            float da_fz = *reinterpret_cast<float*>(g_force_ptrsda[2]);
            float da_mx = *reinterpret_cast<float*>(g_force_ptrsda[3]);
            float da_my = *reinterpret_cast<float*>(g_force_ptrsda[4]);
            float da_mz = *reinterpret_cast<float*>(g_force_ptrsda[5]);
            temp_offset.fx += da_fx;
            temp_offset.fy += da_fy;
            temp_offset.fz += da_fz;
            temp_offset.mx += da_mx;
            temp_offset.my += da_my;
            temp_offset.mz += da_mz;
            float xiao_fx = *reinterpret_cast<float*>(g_force_ptrsxiao[0]);
            float xiao_fy = *reinterpret_cast<float*>(g_force_ptrsxiao[1]);
            float xiao_fz = *reinterpret_cast<float*>(g_force_ptrsxiao[2]);
            float xiao_mx = *reinterpret_cast<float*>(g_force_ptrsxiao[3]);
            float xiao_my = *reinterpret_cast<float*>(g_force_ptrsxiao[4]);
            float xiao_mz = *reinterpret_cast<float*>(g_force_ptrsxiao[5]);
            temp_offset_xiao.fx += xiao_fx;
            temp_offset_xiao.fy += xiao_fy;
            temp_offset_xiao.fz += xiao_fz;
            temp_offset_xiao.mx += xiao_mx;
            temp_offset_xiao.my += xiao_my;
            temp_offset_xiao.mz += xiao_mz;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    g_sensor_offset.fx = temp_offset.fx / sample_count;
    g_sensor_offset.fy = temp_offset.fy / sample_count;
    g_sensor_offset.fz = temp_offset.fz / sample_count;
    g_sensor_offset.mx = temp_offset.mx / sample_count;
    g_sensor_offset.my = temp_offset.my / sample_count;
    g_sensor_offset.mz = temp_offset.mz / sample_count;
    g_sensor_offset_xiao.fx = temp_offset_xiao.fx / sample_count;
    g_sensor_offset_xiao.fy = temp_offset_xiao.fy / sample_count;
    g_sensor_offset_xiao.fz = temp_offset_xiao.fz / sample_count;
    g_sensor_offset_xiao.mx = temp_offset_xiao.mx / sample_count;
    g_sensor_offset_xiao.my = temp_offset_xiao.my / sample_count;
    g_sensor_offset_xiao.mz = temp_offset_xiao.mz / sample_count;
    std::cout << "传感器(大)清零完成! 偏移量 - Fx: " << g_sensor_offset.fx 
              << ", Fy: " << g_sensor_offset.fy 
              << ", Fz: " << g_sensor_offset.fz 
              << ", Mx: " << g_sensor_offset.mx
              << ", My: " << g_sensor_offset.my
              << ", Mz: " << g_sensor_offset.mz << std::endl;
    std::cout << "传感器(小)清零完成! 偏移量 - Fx: " << g_sensor_offset_xiao.fx 
              << ", Fy: " << g_sensor_offset_xiao.fy 
              << ", Fz: " << g_sensor_offset_xiao.fz 
              << ", Mx: " << g_sensor_offset_xiao.mx
              << ", My: " << g_sensor_offset_xiao.my
              << ", Mz: " << g_sensor_offset_xiao.mz << std::endl;
}


// 读取大量程传感器 (原始值)
SensorData read_force_sensor_da_raw() {
    if (!g_is_sensor_ready) return {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double raw_fx = *reinterpret_cast<float*>(g_force_ptrsda[0]);
    double raw_fy = *reinterpret_cast<float*>(g_force_ptrsda[1]);
    double raw_fz = *reinterpret_cast<float*>(g_force_ptrsda[2]);
    double raw_mx = *reinterpret_cast<float*>(g_force_ptrsda[3]);
    double raw_my = *reinterpret_cast<float*>(g_force_ptrsda[4]);
    double raw_mz = *reinterpret_cast<float*>(g_force_ptrsda[5]);
    return { raw_fx, raw_fy, raw_fz, raw_mx, raw_my, raw_mz };
}

// 读取小量程传感器 (原始值)
SensorData read_force_sensor_xiao_raw() {
    if (!g_is_sensor_ready) return {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    float rawx_fx = *reinterpret_cast<float*>(g_force_ptrsxiao[0]);
    float rawx_fy = *reinterpret_cast<float*>(g_force_ptrsxiao[1]);
    float rawx_fz = *reinterpret_cast<float*>(g_force_ptrsxiao[2]);
    float rawx_mx = *reinterpret_cast<float*>(g_force_ptrsxiao[3]);
    float rawx_my = *reinterpret_cast<float*>(g_force_ptrsxiao[4]);
    float rawx_mz = *reinterpret_cast<float*>(g_force_ptrsxiao[5]);
    return { (double)rawx_fx, (double)rawx_fy, (double)rawx_fz, (double)rawx_mx, (double)rawx_my, (double)rawx_mz };
}

// 读取大量程传感器 (减去偏移)
SensorData read_force_sensor_da() {
    SensorData raw = read_force_sensor_da_raw();
    return { 
        raw.fx - g_sensor_offset.fx, 
        raw.fy - g_sensor_offset.fy, 
        raw.fz - g_sensor_offset.fz, 
        raw.mx - g_sensor_offset.mx, 
        raw.my - g_sensor_offset.my, 
        raw.mz - g_sensor_offset.mz 
    };
}

// 读取小量程传感器 (减去偏移)
SensorData read_force_sensor_xiao() {
    SensorData raw = read_force_sensor_xiao_raw();
    return { 
        raw.fx - g_sensor_offset_xiao.fx, 
        raw.fy - g_sensor_offset_xiao.fy, 
        raw.fz - g_sensor_offset_xiao.fz, 
        raw.mx - g_sensor_offset_xiao.mx, 
        raw.my - g_sensor_offset_xiao.my, 
        raw.mz - g_sensor_offset_xiao.mz 
    };
}



// 获取当前状态
MyRobotState read_robot_full_state() {
    MyRobotState state = {0};
    NRC_Position mcsPos, acsPos;
    
   
    if (NRC_GetCurrentPos(NRC_COORD::NRC_MCS, mcsPos) == 0) {
        state.x  = mcsPos.pos[0] / 1000.0;
        state.y  = mcsPos.pos[1] / 1000.0;
        state.z  = mcsPos.pos[2] / 1000.0;
        state.rz = mcsPos.pos[5]; // 弧度
    }
   
    if (NRC_GetCurrentPos(NRC_COORD::NRC_ACS, acsPos) == 0) {
        
        // std::cout << "Current ACS Position - theta2: " << acsPos.pos[0] << " deg, 立柱高度: "<< acsPos.pos[1] << "mm, 伸缩长度：" << acsPos.pos[2] << "mm,末端角度 " << acsPos.pos[3] << " deg" << std::endl;    
        state.theta2 = acsPos.pos[0] * (M_PI / 180.0);
        state.theta4 = acsPos.pos[3] * (M_PI / 180.0);
    }
    return state;
}


bool perform_ik(const NRC_Position& reference_acs,
               double x_m,
               double y_m,
               double z_m,
               double rz_rad,
               NRC_Position& result_joints) {
    
    NRC_Position posMCS(NRC_COORD::NRC_MCS,
                        x_m * 1000.0,
                        y_m * 1000.0,
                        z_m * 1000.0,
                        3.14159,
                        0,
                        rz_rad);

    
    NRC_Position referencePos = reference_acs;
    return (NRC_MCStoACS(referencePos, posMCS, result_joints) == 0);
}



int main() {


    //系统启动
    SystemStartup();

    
    std::vector<double> vMax = {60, 60, 60, 60, 20, 20, 20};  
    std::vector<double> avMax = {1500, 1500, 1500, 2000, 2000, 2000, 2000}; 
    std::vector<double> jMax = {1500, 2000, 2000, 2000, 2000, 2000, 2000}; 

	
	NRC_RKG_Open(vMax, avMax, jMax); 

    
    double dt =1.0/900.0; 
    Admittance4::Vec4 M = {130.0, 130.0, 110.0, 5}; 
    Admittance4::Vec4 D = {3000.0, 2000.0, 5000.0, 120};
    Admittance4::Vec4 K = {0.0, 0.0, 0.0, 0.0};
    Admittance4 controller(M, D, K, dt);

    //若初始化传感器失败，退出程序
    if (!init_force_sensor_mapping()) {
    std::cout << "错误: 力传感器 PDO 映射失败！请检查 EtherCAT 连接或 0x6030 对象字典配置。" << std::endl;
    return -1; 
    }
    
    // 传感器清零
    zero_force_sensor();
    

    MyRobotState init_s = read_robot_full_state();

    
    double base_x = init_s.x;
    double base_y = init_s.y;
    double base_z = init_s.z;
    double base_t4 = init_s.theta4;
    double initial_total_rz = init_s.rz;
    

    Admittance4::Vec4 zero_state = {0,0,0,0};
    controller.set_state(zero_state, zero_state);


    //控制模式定义 
    enum ControlMode {
        MODE_POS, // 位置模式：只移不转 
        MODE_ROT  // 旋转模式：只转不移 
    };
    ControlMode current_mode = MODE_POS; // 默认位置模式
    bool last_button_state = false;      // 按钮状态记忆
   


    std::cout << "工具坐标系导纳控制启动... (使用IO 1.1 切换模式：1=旋转, 0=位置)" << std::endl;


    // 用于记录上一帧的导纳位移状态，以便切换时清零速度
    Admittance4::Vec4 last_target_tool = {0,0,0,0};
    Admittance4::Vec4 active_pos_lock = {0,0,0,0}; 
    double locked_rz = 0.0; 

	// 启动力传感器六维数据同步线程（小量程7-12，大量程13-18）
    std::thread force_feedback_thread([](){
        while (true) {
            // 使用 _raw 函数获取原始数据
            SensorData data_xiao = read_force_sensor_xiao_raw();
            NRC_SetDoubleVar(7, data_xiao.fx);
            NRC_SetDoubleVar(8, data_xiao.fy);
            NRC_SetDoubleVar(9, data_xiao.fz);
            NRC_SetDoubleVar(10, data_xiao.mx);
            NRC_SetDoubleVar(11, data_xiao.my);
            NRC_SetDoubleVar(12, data_xiao.mz);
            
            SensorData data_da = read_force_sensor_da_raw();
            NRC_SetDoubleVar(13, data_da.fx);
            NRC_SetDoubleVar(14, data_da.fy);
            NRC_SetDoubleVar(15, data_da.fz);
            NRC_SetDoubleVar(16, data_da.mx);
            NRC_SetDoubleVar(17, data_da.my);
            NRC_SetDoubleVar(18, data_da.mz);
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
    });
    force_feedback_thread.detach();

    bool last_force_switch = false; 
    bool last_zero_switch = false;
    bool last_remote_switch =false;
    bool last_servo_ready = false;

    
	while (true)  
    {
        int servo_ready_val = NRC_ReadBoolVar(4);
        bool servo_ready_on = (servo_ready_val == 1);
        // 上升沿：使能伺服
        if(servo_ready_on && !last_servo_ready)
        {   
            NRC_SetServoReadyStatus(1);
            logger.log("Servo ready status set to 1\n");
            NRC_ServoEnable();
        }
        // 下降沿：禁止伺服，下使能
        if(!servo_ready_on && last_servo_ready)
        {
            NRC_ServoDisable();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            NRC_SetServoReadyStatus(0);
            logger.log("Servo ready status set to 0, servo disabled\n");
        }




        int remote_sw_val = NRC_ReadBoolVar(3);
        bool remote_sw_on = (remote_sw_val == 1);
        if(remote_sw_on && !last_remote_switch)
        {   
            
            int ret = NRC_SetOperationMode(NRC_REMOTE_);
            std::cout <<"远程切换返回值"<< ret <<std::endl;
            // logger.log("远程切换返回值: " + std::to_string(ret));
        }
        else if(!remote_sw_on && last_remote_switch)
        {
            int ret = NRC_SetOperationMode(NRC_TEACH_);
            std::cout << "示教切换返回值" << ret << std::endl;
        }
        last_remote_switch = remote_sw_on;
        

        // 执行清零
        int zero_sw_val = NRC_ReadBoolVar(2);
        bool zero_sw_on = (zero_sw_val == 1);
        if (zero_sw_on && !last_zero_switch) {
             zero_force_sensor();
             std::cout << " 力传感器清零完成。" << std::endl;
        }
        last_zero_switch = zero_sw_on;

        int force_sw_val = NRC_ReadBoolVar(1);
        bool force_sw_on = (force_sw_val == 1);
        
        // 执行初始化
        if (force_sw_on && !last_force_switch) {
            

            NRC_SetServoReadyStatus(1);
            
            NRC_PowerOn();

            
            
            // 1. 传感器清零
            zero_force_sensor();
            
            // 2. 重新获取当前位置作为基准
            init_s = read_robot_full_state();
            base_x = init_s.x;
            base_y = init_s.y;
            base_z = init_s.z;
            base_t4 = init_s.theta4;
            initial_total_rz = init_s.rz;
            
            // 3. 重置控制器状态
            Admittance4::Vec4 zero_st = {0,0,0,0};
            controller.set_state(zero_st, zero_st);
            last_target_tool = {0,0,0,0};
            active_pos_lock = {0,0,0,0};
            
            std::cout << ">>> 初始化完成，开始力控..." << std::endl;
        }
        
        
        if (!force_sw_on && last_force_switch) {
            std::cout << ">>> 检测到开关关闭，执行伺服下电..." << std::endl;
            // 关闭透传
            NRC_RKG_Stop();
            //伺服下电
            NRC_PowerOff();
        }

        last_force_switch = force_sw_on;

        // 如果开关未开启，跳过力控逻辑，保持当前状态
        if (!force_sw_on) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        // // --- 读取反馈 ---
        MyRobotState curr_s = read_robot_full_state();
        NRC_Position reference_acs;
        NRC_GetCurrentPos(NRC_COORD::NRC_ACS, reference_acs);


        // 读取偏移处理后的力数据 (用于导纳控制)
        SensorData ft = read_force_sensor_da();
        // SensorData fxiao = read_force_sensor_xiao();
        std::cout << "Force Sensor Readings - Fx: " << ft.fx << " N, Fy: " << ft.fy << " N, Fz: " << ft.fz << " N, Mz: " << ft.mz << " Nm" << std::endl;
        // std::cout << "Force Sensor Readings - Fx: " << fxiao.fx << " N, Fy: " << fxiao.fy << " N, Fz: " << fxiao.fz << " N, Mz: " << fxiao.mz << " Nm" << std::endl;
        
        
        if (std::abs(ft.fx) < 2.0) ft.fx = 0.0;
        if (std::abs(ft.fy) < 2.0) ft.fy = 0.0;
        if (std::abs(ft.fz) < 2.0) ft.fz = 0.0;
        if (std::abs(ft.mz) < 0.5) ft.mz = 0.0;

        
        static Admittance4::Vec4 last_F = {0,0,0,0};
        double alpha = 0.15; 
        ft.fx = alpha * ft.fx + (1 - alpha) * last_F[0];
        ft.fy = alpha * ft.fy + (1 - alpha) * last_F[1];
        last_F = {ft.fx, ft.fy, ft.fz, ft.mz};


        // 状态 1 = 旋转模式 (MODE_ROT) - 锁定位置，只允许旋转
        // 状态 0 = 位置模式 (MODE_POS) - 锁定旋转，只允许移动
        int io_state = NRC_ReadDigInByBoard(1, 1);
        
        ControlMode target_mode = (io_state == 1) ? MODE_ROT : MODE_POS;

        if (target_mode != current_mode) {
             current_mode = target_mode;
             
             if (current_mode == MODE_ROT) {
                 std::cout << "\r\n>>> [IO触发] 模式切换: 旋转模式 (位置锁定)" << std::endl;
             } else {
                 std::cout << "\r\n>>> [IO触发] 模式切换: 位置模式 (旋转锁定)" << std::endl;
             }
             
            //切换模式瞬间，重置导纳内部速度为0，消除惯性漂移
            Admittance4::Vec4 zero_vel = {0,0,0,0};
            controller.set_state(last_target_tool, zero_vel);
            
            // 记录进入旋转模式的那一刻的位移，作为锁定位
            if (current_mode == MODE_ROT) {
                active_pos_lock = last_target_tool; 
                locked_rz = curr_s.rz; // 锁定当前角度，防止转动时带动工具系位移投影变化
            }
        }
    


        if (current_mode == MODE_POS) {
            ft.mz = 0.0; // 位置模式：忽略力矩，防止误触转动
        } 
 

        Admittance4::Vec4 F_input = {ft.fx, ft.fy, ft.fz, ft.mz};
        
        auto result = controller.update(F_input);
        Admittance4::Vec4 target_tool = result.first; 
        
        // 修正输出
        if (current_mode == MODE_ROT) {
            
            target_tool[0] = active_pos_lock[0];
            target_tool[1] = active_pos_lock[1];
            target_tool[2] = active_pos_lock[2];
            // 只允许 target_tool[3] (旋转) 变化
        } else if (current_mode == MODE_POS) {
          
        }

        last_target_tool = target_tool;

        
        
        double angle_for_proj = (current_mode == MODE_ROT) ? -locked_rz : -curr_s.rz;
        double c = std::cos(angle_for_proj);
        double s = std::sin(angle_for_proj);

        // 工具系到基座系位移
        double dx_base = target_tool[0] * c - target_tool[1] * s;
        double dy_base = target_tool[0] * s + target_tool[1] * c;
        
        // 计算基座系下的绝对目标位置
        double target_x_abs = base_x + dx_base;
        double target_y_abs = base_y + dy_base;
        double target_z_abs = base_z + target_tool[2]; 

  
        
       
        NRC_Position ik_joints;
        // 锁定的初始总姿态
        if (perform_ik(reference_acs, target_x_abs, target_y_abs, target_z_abs, initial_total_rz, ik_joints)) {
            
            double cmd_t2 = ik_joints.pos[0];
            double cmd_d1 = ik_joints.pos[1];
            double cmd_d3 = ik_joints.pos[2];
            // 关节4继承工具系的姿态变化
            double cmd_t4 = base_t4+target_tool[3];

            // std::cout << "位置 - d1立柱高度: " << cmd_d1 << " mm, theta2: " << cmd_t2  << " 度, d3伸缩长度: " << cmd_d3 << " mm, theta4末端角度: " << cmd_t4 * 180.0/M_PI << " 度" << std::endl;
			target_joints[0] = cmd_t2;
			target_joints[1] = cmd_d1; 
			target_joints[2] = cmd_d3; 
			target_joints[3] = cmd_t4 * 180.0 / M_PI; 
			target_joints[4] = 0;
			target_joints[5] = 0;
			target_joints[6] = 0;
			NRC_Set_ServoJ_Pos(target_joints);
        
        }
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    // 关闭透传
    NRC_RKG_Stop();
    return 0;
}
