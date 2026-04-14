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