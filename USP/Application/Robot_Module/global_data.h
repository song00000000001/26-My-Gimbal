#pragma once

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


extern uint32_t vision_last_recv_time ; 
extern VisionRecvData_t vision_recv_pack;

extern motor_ctrl_driver motor_ctrl;//电机控制驱动实例

//system state enum
typedef enum{
    idle = 0,
    wait_start,
    small_energy,
    big_energy,
    success
}EnergySystemMode_t;

//big energy state enum
typedef enum{
    BE_GENERATE_TARGET = 0,
    BE_WAIT_HIT_1,
    BE_WAIT_HIT_2,
    BE_END
}BigEnergyState_t;

//small energy state enum
typedef enum{
    SE_GENERATE_TARGET = 0,
    SE_WAIT_HIT,
    SE_END
}SmallEnergyState_t;

//颜色枚举
typedef enum 
{
    color_off = 0,
    color_red,
    color_blue,
    color_hit_red,
    color_hit_blue
}light_color_enum;

typedef enum 
{
    main_arm_outside = 0,
    main_arm_middle,
    main_arm_inside,
    sub_arm_left,
    sub_arm_right
}ligntarm_name_enum;

// 命令类型定义
enum FanCmdType {
    FAN_CMD_RESET = 0x01,       // 复位/熄灭
    FAN_CMD_SELECT = 0x02,// 正常点亮
    FAN_CMD_HIT = 0x03,   // 击打闪烁+中心十字线亮起
};

// 全局状态结构体 - 用于Debug控制和系统状态管理
typedef struct {
    // --- 控制参数 (Debug可修改) ---
    uint8_t target_mode;    // 0:停止/待机, 1:激活, 2. 小能量机关, 3:大能量机关 ,4: 连续小能量机关,5: 连续大能量机关
    EnergySystemMode_t  SysMode;       // 0:停止/待机, 1:小能量机关, 2:大能量机关
    light_color_enum  TargetColor;   // 0:Red, 1:Blue
    uint8_t  set_color; // 当前颜色状态(与反馈保持一致),0:Red, 1:Blue
    float    SmallEnergy_Speed; // 小符固定速度
    float    BigEnergy_A;   // 大符正弦 A
    float    BigEnergy_W;   // 大符正弦 Omega
    
    // --- 运行状态 (只读) ---
    uint8_t  CurrentHitID;  // 当前被击中的ID (反馈)
    uint8_t  IsHit;         // 是否被击中标记
    float    TargetSpeed;   // 当前计算目标速度
    float    RealSpeed;     // 当前电机反馈速度
    uint16_t CurrentHitRound;      // 当前击中环数
    
    // --- 小能量机关状态变量  ---
    uint8_t  SE_TargetID;   // 当前目标ID (1-5)
    uint8_t  SE_Group;      // 当前轮数 (0，1-5)
    SmallEnergyState_t  SE_State;      // 状态机: 0:生成, 1:等待击打, 2:结束
    uint32_t SE_StateTimer; // 状态计时器

    // --- 大能量机关状态变量  ---
    uint8_t  BE_Group;      // 当前轮数 (0，1-5)
    uint8_t  BE_Targets[2]; // 当前两个目标ID (1-5)
    BigEnergyState_t  BE_State;      // 状态机: 0:生成, 1:等待首击, 2:等待连击, 3:结束
    uint32_t BE_StateTimer; // 状态计时器
} EnergySystemState_t;

extern EnergySystemState_t g_SystemState;

// 通信包结构体 (4字节定长)
#pragma pack(1)
typedef struct {
    uint8_t header;       // 固定为 0xA5
    uint8_t cmd;          // 见 FanCmdType
    uint8_t color;        // 0:Blue, 1:Red (与 Current_color 保持一致)
    //uint8_t target_id;    // 目标装甲板ID (虽然物理线路已区分，但保留此字段用于校验或显示)
    uint8_t current_stage;// 新增：当前击打阶段/组数 (0-5)，用于大符阶梯式亮灯效果
} FanPacket_t;
#pragma pack()

void FanFeedbackProcess(CAN_COB &CAN_RxMsg);

#ifdef __cplusplus
}
#endif