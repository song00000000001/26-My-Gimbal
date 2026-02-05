#include "global_data.h"
#include "robot_config.h"
#include "internal.h"

VisionRecvData_t vision_recv_pack;//电视接收包
uint32_t vision_last_recv_time = 0; // 视觉最后接收时间,communication.cpp中更新,server发送时使用

Debug_Data_t Debugger;//调试数据结构体实例化

#ifdef INCLUDE_uxTaskGetStackHighWaterMark
stack_remain_t Stack_Remain={255};
#endif

motor_ctrl_driver motor_ctrl(1);//电机控制驱动实例