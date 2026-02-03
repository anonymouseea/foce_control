#include "ControlLoop.h"

#include <array>
#include <chrono>
#include <cmath>
#include <thread>
#include <time.h>

#include "Admittance.h"
#include "RobotUtils.h"
#include "nrcAPI.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

void RunControlLoop(AsyncLogger& logger, std::atomic<bool>& running) {
    Admittance4 controller({130.0, 130.0, 120.0, 5},
                           {3000.0, 2500.0, 5000.0, 120},
                           {0, 0, 0, 0}, 0.001);

    MyRobotState init_s = read_robot_full_state();
    double base_x = init_s.x, base_y = init_s.y, base_z = init_s.z;
    double base_t4 = init_s.theta4, initial_total_rz = init_s.rz;

    enum ControlMode { MODE_POS, MODE_ROT };
    ControlMode current_mode = MODE_POS;
    Admittance4::Vec4 last_target_tool = {0, 0, 0, 0};
    Admittance4::Vec4 active_pos_lock = {0, 0, 0, 0};
    double locked_rz = 0.0;
    std::array<double, 7> target_joints{};

    std::thread force_feedback_thread([&running]() {
        struct sched_param param;
        param.sched_priority = 0;
        pthread_setschedparam(pthread_self(), SCHED_OTHER, &param);
        while (running) {
            SensorData data_xiao = read_force_sensor_xiao_raw();
            NRC_SetDoubleVar(7, data_xiao.fx); NRC_SetDoubleVar(8, data_xiao.fy); NRC_SetDoubleVar(9, data_xiao.fz);
            NRC_SetDoubleVar(10, data_xiao.mx); NRC_SetDoubleVar(11, data_xiao.my); NRC_SetDoubleVar(12, data_xiao.mz);

            SensorData data_da = read_force_sensor_da_raw();
            NRC_SetDoubleVar(13, data_da.fx); NRC_SetDoubleVar(14, data_da.fy); NRC_SetDoubleVar(15, data_da.fz);
            NRC_SetDoubleVar(16, data_da.mx); NRC_SetDoubleVar(17, data_da.my); NRC_SetDoubleVar(18, data_da.mz);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    });
    force_feedback_thread.detach();

    bool last_force_switch = false, last_zero_switch = false;

    // --- 高精度硬实时循环起始 ---
    struct timespec next_p;
    clock_gettime(CLOCK_MONOTONIC, &next_p);

    SensorData ft{};
    double dead_zone_f = 0.0, dead_zone_m = 0.0;

    while (running) {
        // 判断当前是否力控开启，动态调整周期
        bool force_on = (NRC_ReadBoolVar(1) == 1);
        int cycle_ns = force_on ? 1000000 : 10000000; // 力控时1ms, 否则10ms

        // 设置下一个唤醒时间点
        next_p.tv_nsec += cycle_ns;
        while (next_p.tv_nsec >= 1000000000L) { next_p.tv_nsec -= 1000000000L; next_p.tv_sec++; }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_p, NULL);

        // 传感器清零逻辑
        bool zero_on = (NRC_ReadBoolVar(2) == 1);
        if (zero_on && !last_zero_switch) { zero_force_sensor(logger); }
        last_zero_switch = zero_on;

        // 控开关逻辑
        force_on = (NRC_ReadBoolVar(1) == 1); // 再次获取，保证后续逻辑一致
        if (force_on && !last_force_switch) {
            logger.log("[CONTROL] Force control enabled\n");
            NRC_ClearAllError();
            zero_force_sensor(logger);
            NRC_SetServoReadyStatus(1);
            NRC_PowerOn();
            init_s = read_robot_full_state();
            base_x = init_s.x; base_y = init_s.y; base_z = init_s.z; base_t4 = init_s.theta4; initial_total_rz = init_s.rz;
            controller.set_state({0,0,0,0}, {0,0,0,0});
            last_target_tool = {0,0,0,0};
            NRC_RKG_Open({40,40,60,60,20,20,20},{1500,1500,1500,2000,2000,2000,2000},{2000,2000,2000,2000,2000,2000,2000});
        }

        // 关闭力控
        if (!force_on && last_force_switch) { logger.log("[CONTROL] Force control disabled\n"); NRC_RKG_Stop(); NRC_PowerOff(); }
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
            //限位保护
            if(target_joints[0]<-44 || target_joints[0]>44 ||
               target_joints[1]<-840 || target_joints[1]>1148 ||
               target_joints[2]<5 || target_joints[2]>848 ||
               target_joints[3]<-60 || target_joints[3]>60 )
            {
                NRC_SetBoolVar(4,1); // 触发限位报警
                NRC_SetBoolVar(1,0); // 关闭力控开关
                logger.log("[ERROR] 关节超限，力控关闭.\n");
                continue;
            }
            NRC_Set_ServoJ_Pos(target_joints);
        }
    }
}
