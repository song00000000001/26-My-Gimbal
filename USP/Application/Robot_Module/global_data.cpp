#include "global_data.h"
#include "robot_config.h"
#include "internal.h"

VisionRecvData_t vision_recv_pack;//电视接收包
uint32_t vision_last_recv_time = 0; // 视觉最后接收时间,communication.cpp中更新,server发送时使用

Debug_Data_t Debugger;//调试数据结构体实例化

#ifdef INCLUDE_uxTaskGetStackHighWaterMark
stack_remain_t Stack_Remain={255};
#endif

motor_ctrl_driver motor_ctrl(mymotor_id);//电机控制驱动实例


//全局变量定义
// 系统状态实例
EnergySystemState_t g_SystemState = {
    .SysMode = idle,                // 默认待机
    .TargetColor = color_red,       // 默认红色
    .set_color =  0,              // 默认红色
    .SmallEnergy_Speed = 1.0f,      // 默认倍率
    .BigEnergy_A = 0.9125f,         // 默认大符参数 (0.780 + 1.045)/2
    .BigEnergy_W = 1.942f,          // 默认大符参数 (1.884 + 2.000)/2
    .BE_Group = 0,                  // 初始轮数
    .BE_State = BE_GENERATE_TARGET                   // 初始状态
};
