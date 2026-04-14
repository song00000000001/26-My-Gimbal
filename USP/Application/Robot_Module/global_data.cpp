#include "global_data.h"
#include "robot_config.h"
#include "internal.h"


Debug_Data_t Debugger={};//调试数据结构体实例化

#if STACK_REMAIN_MONITER_ENABLE
stack_remain_t Stack_Remain={255};
#endif


// 电机实例
BenMoMotor g_motors[MOTOR_COUNT] = { BenMoMotor(1), BenMoMotor(2) };

//临时变量
MyPid_Struct g_pid[MOTOR_COUNT]; // PID控制器实例数组
MyPid_Debug_Struct g_pid_debug[MOTOR_COUNT]; // PID参数实例数组

float g_imu_angle_deg[2] = {0.0f}; // 来自IMU的当前角度反馈，单位为度
float g_imu_gyro_dps[2] = {0.0f}; // 来自IMU的角速度反馈，单位为度每秒

MotorSpeedCtrl g_speed_ctrl[MOTOR_COUNT];
MotorSpeedCtrlParam g_speed_ctrl_param = {
    .stop_rpm_th = 5.0f,
    .stop_ref_th = 5.0f,
    .starting_rpm_th = 5.0f,
    .running_rpm_th = 8.0f,
    .running_hold_ms = 30U,
    .startup_timeout_ms = 500U,
    .dir_ref_th = 10.0f,
    .dir_iq_th = 0.01f,
    .i_start_min = 0.09f,
    .i_start_boost = 0.04f,
    .i_fric_run = 0.04f,//太大会过冲
    .iq_cmd_limit = 0.6f,
};

BenMoMotor* motor_observer; // 选择一个电机作为观察对象
MyPid_Struct *pid_observer; // 选择一个PID实例作为观察对象
MyPid_Debug_Struct *pid_debug_observer; // 选择一个PID参数实例作为观察对象
MotorSpeedCtrl *spd_ctrl_observer; // 选择一个速度控制器实例作为观察对象
MotorSpeedCtrlParam *spd_ctrl_param_observer; // 选择一个速度控制器参数实例作为观察对象
