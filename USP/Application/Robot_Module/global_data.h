#pragma once

#include "internal.h"
#include "motor_ctrl_driver.h"
#include "remote_ctrl_driver.h"

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

#ifdef STACK_REMAIN_MONITER_ENABLE
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

typedef enum{
    tar_stop = 0,
    tar_start,
    tar_small_energy_signle,
    tar_big_energy_single,
    tar_small_energy_continue,
    tar_big_energy_continue
}EnergyTargetMode_t;

// 命令类型定义
enum FanCmdType {
    FAN_CMD_RESET = 0x01,       // 复位/熄灭
    FAN_CMD_SELECT = 0x02,// 正常点亮
    FAN_CMD_HIT = 0x03,   // 击打闪烁+中心十字线亮起
};

typedef struct {
    EnergyTargetMode_t target_mode;    // 0:停止/待机, 1:激活, 2. 小能量机关, 3:大能量机关 ,4: 连续小能量机关,5: 连续大能量机关
    light_color_enum  TargetColor;   // 0:Red, 1:Blue
    float    SmallEnergy_Speed; // 小符固定速度
    float    BigEnergy_A;   // 大符正弦 A
    float    BigEnergy_W;   // 大符正弦 Omega
}target_ctrl_t;
extern target_ctrl_t g_TargetCtrl;

//小能量机关状态结构体
typedef struct{
    // --- 小能量机关状态变量  ---
    SmallEnergyState_t  SE_State;      // 状态机: 0:生成, 1:等待击打, 2:结束
    uint8_t  SE_TargetID; // 当前目标ID (1-5)
    uint8_t  SE_Group;      // 当前轮数 (0，1-5)
    uint8_t  SE_TargetID_GROUP[5];   // 当前目标序列组 
    uint32_t SE_StateTimer; // 状态计时器
    uint8_t  SE_Scores;
}SmallEnergyStateData_t;

//大能量机关状态结构体
typedef struct{
    // --- 大能量机关状态变量  ---
    uint8_t  BE_Group;      // 当前轮数 (0，1-5)
    uint8_t  BE_Targets[2]; // 当前两个目标ID (1-5)
    BigEnergyState_t  BE_State;      // 状态机: 0:生成, 1:等待首击, 2:等待连击, 3:结束
    uint32_t BE_StateTimer; // 状态计时器
    uint8_t  BE_Scores;
    uint8_t  BE_ActivedArms; // 当前激活的装甲板数量 (1-10)
}BigEnergyStateData_t;

// 全局状态结构体 - 用于Debug控制和系统状态管理
typedef struct {
    EnergySystemMode_t  SysMode;       // 0:停止/待机, 1:小能量机关, 2:大能量机关
    SmallEnergyStateData_t SE_StateData; // 小能量机关状态数据
    BigEnergyStateData_t BE_StateData; // 大能量机关状态数据
    uint8_t  CurrentHitID;  // 当前被击中的ID (反馈)
    uint8_t  CurrentHitScores; // 当前得分 (根据击打情况计算，供上位机显示)
    float    TargetSpeed;   // 当前计算目标速度 (供上位机显示)
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

void Ctrl_All_Armors(FanCmdType cmd, light_color_enum color, uint8_t stage) ;
void lightSuccessFlash(int8_t num, light_color_enum color);
void small_energy_logic();
void big_energy_logic();
void debug_simulate_hit_f() ;//调试函数，模拟一次一定正确且环数随机的击打事件

#pragma pack(1)
typedef struct {
    uint8_t ctrl_header;
    uint8_t ctrl_content;
} UpperCtrlPacket_t;
#pragma pack()
extern UpperCtrlPacket_t upper_ctrl_packet;
#ifdef __cplusplus
}
#endif