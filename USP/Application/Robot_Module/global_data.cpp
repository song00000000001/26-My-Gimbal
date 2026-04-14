#include "global_data.h"
#include "robot_config.h"
#include "internal.h"


Debug_Data_t Debugger={};//调试数据结构体实例化

#if STACK_REMAIN_MONITER_ENABLE
stack_remain_t Stack_Remain={255};
#endif

//全局变量定义
// 系统状态实例
EnergySystemState_t g_SystemState ;
EnergySystemState_t g_TargetCtrl;

// 电机实例
BenMoMotor gimbal_motors[MOTOR_COUNT] = { BenMoMotor(1), BenMoMotor(2) };

//临时变量
MyPid gimbal_pid_pos[MOTOR_COUNT]={}; // PID控制器实例数组
MyPid gimbal_pid_spd[MOTOR_COUNT]={}; // PID控制器实例数组
MyPid gimbal_pid_cur[MOTOR_COUNT]={}; // PID控制器实例数组
MyPidParam gimbal_pid_param_pos[MOTOR_COUNT]={}; // PID参数实例数组
MyPidParam gimbal_pid_param_spd[MOTOR_COUNT]={}; // PID参数实例数组
MyPidParam gimbal_pid_param_cur[MOTOR_COUNT]={}; // PID参数实例数组
float hold_angle_deg[2] = {34.0f, 0.0f}; // 固定的目标位置，单位为度
float imu_angle_deg[2] = {0.0f}; // 来自IMU的当前角度反馈，单位为度
float imu_gyro_dps[2] = {0.0f}; // 来自IMU的角速度反馈，单位为度每秒