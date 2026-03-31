#pragma once

#include "internal.h"
//#include "motor_ctrl_driver.h"
//#include "remote_ctrl_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum{
    debug_idle = 0,
    debug_mtvofa_monitor,
    debug_mpuvoda_monitor
}Debug_Mode_e;

//调试数据结构体
typedef struct {
    // 标志位
    Debug_Mode_e enable_debug_mode; // 在watch窗口改为true以进入调试模式(配合遥控器)
    bool Debug_simulate_hit;// 模拟击打（进入调试模式后，在watch窗口改为true以模拟一次击打事件，配合遥控器）
} Debug_Data_t;
extern Debug_Data_t Debugger;  

#if STACK_REMAIN_MONITER_ENABLE
//定义栈剩余空间记录结构体
typedef struct 
{
    uint16_t FS_I6X_stack_remain;
    uint16_t motor_ctrl_stack_remain;
    uint16_t state_machine_stack_remain;
    uint16_t armer_ctrl_stack_remain;
    uint16_t uart_comm_stack_remain;
    uint16_t debug_send_stack_remain;
    uint16_t vision_comm_stack_remain;
    uint16_t can_comm_stack_remain;

}stack_remain_t;
extern stack_remain_t Stack_Remain;

//把栈水位获取封装成宏
#define StackWaterMark_Get(x)  \
    do{ \
        Stack_Remain.x##_stack_remain = uxTaskGetStackHighWaterMark(NULL); \
    }while(0)

#endif

//system state enum
typedef enum{
    idle = 0,
}EnergySystemMode_t;

// 全局状态结构体 - 用于Debug控制和系统状态管理
typedef struct {
    EnergySystemMode_t  SysMode;       // 0:停止/待机, 1:小能量机关, 2:大能量机关
	//others
} EnergySystemState_t;
extern EnergySystemState_t g_SystemState;

// 通信包结构体 (4字节定长)
#pragma pack(1)
typedef struct {
    uint8_t header;       // 固定为 0xA5
    uint8_t cmd;          
    uint8_t data;   
} My_Packet_t;
#pragma pack()

#ifdef __cplusplus
}
#endif