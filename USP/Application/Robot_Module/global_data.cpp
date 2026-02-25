#include "global_data.h"
#include "robot_config.h"
#include "internal.h"

Debug_Data_t Debugger;//调试数据结构体实例化

#ifdef INCLUDE_uxTaskGetStackHighWaterMark
stack_remain_t Stack_Remain={255};
#endif

motor_ctrl_driver motor_ctrl(mymotor_id);//电机控制驱动实例

UpperCtrlPacket_t upper_ctrl_packet;// 上位机控制包实例

//全局变量定义
// 系统状态实例
EnergySystemState_t g_SystemState = {
    .SysMode = idle,                // 默认待机
    .BE_Group = 0,                  // 初始轮数
    .BE_State = BE_GENERATE_TARGET,                   // 初始状态
    // .CurrentHitID = 0,              // 初始无击打
    // .TargetSpeed = 0.0f,           // 初始目标速度
    // .RealSpeed = 0.0f,             // 初始实际速度
    // .SE_TargetID_GROUP = {0},      // 初始小能量目标序列
    // .SE_TargetID = 0,              // 初始小能量目标ID
};
target_ctrl_t g_TargetCtrl={
    .target_mode = tar_stop,        // 默认停止/待机
    .TargetColor = color_red,      // 默认红色
    .SmallEnergy_Speed = 1.0f,     // 默认倍率
    .BigEnergy_A = 0.9125f,        // 默认大符参数 (0.780 + 1.045)/2
    .BigEnergy_W = 1.942f          // 默认大符参数 (1.884 + 2.000)/2
};