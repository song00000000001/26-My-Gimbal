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
BenMoMotor BenMo_Motor_1(1); // 电机ID 1
BenMoMotor BenMo_Motor_2(2); // 电机ID 2