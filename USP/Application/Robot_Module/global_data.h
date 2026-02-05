#pragma once

#include "robot_types.h"
#include "internal.h"
#include "motor_ctrl_driver.h"
#include "remote_ctrl_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

#pragma pack(1)
    struct VisionRecvData_t
    {
        uint8_t target_mode;
		uint8_t ros=3;
        float target_yaw;
        float pilot_translation;
        uint8_t end;
    };
#pragma pack()



//调试数据结构体
typedef struct {
    // 标志位
    uint8_t enable_debug_mode; // 在watch窗口改为true以进入调试模式(配合遥控器)
} Debug_Data_t;
extern Debug_Data_t Debugger;  

#ifdef INCLUDE_uxTaskGetStackHighWaterMark
//定义栈剩余空间记录结构体
typedef struct 
{
    uint16_t FS_I6X_stack_remain;
    uint16_t debug_send_stack_remain;

}stack_remain_t;
extern stack_remain_t Stack_Remain;
#endif


extern uint32_t vision_last_recv_time ; 
extern VisionRecvData_t vision_recv_pack;

extern motor_ctrl_driver motor_ctrl;//电机控制驱动实例


#ifdef __cplusplus
}
#endif