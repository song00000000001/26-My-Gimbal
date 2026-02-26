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
EnergySystemState_t g_SystemState ;
target_ctrl_t g_TargetCtrl={
    .target_mode = tar_small_energy_signle,        // 默认停止/待机
    .TargetColor = color_red,      // 默认红色
    .SmallEnergy_Speed = 1.0f,     // 默认倍率
    .BigEnergy_A = 0.9125f,        // 默认大符参数 (0.780 + 1.045)/2
    .BigEnergy_W = 1.942f          // 默认大符参数 (1.884 + 2.000)/2
};