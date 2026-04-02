#include "global_data.h"
#include "robot_config.h"
#include "internal.h"


Debug_Data_t Debugger;//调试数据结构体实例化

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
float motor_target_position[MOTOR_COUNT] = {0.0f}; // 目标位置，单位为度
MyPid gimbal_pid[MOTOR_COUNT]={}; // PID控制器实例数组
float motor_cmd_deg[MOTOR_COUNT] = {0.0f}; // 电机控制指令，单位为度
float hold_angle_deg[2] = {180.0f, 180.0f}; // 固定的目标位置，单位为度
float imu_angle_deg[2] = {0.0f}; // 来自IMU的当前角度反馈，单位为度
float imu_gyro_dps[2] = {0.0f}; // 来自IMU的角速度反馈，单位为度每秒