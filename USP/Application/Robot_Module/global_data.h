#pragma once

#include "stdint.h"
#include "Yaw_control.h"
#include "launcher_driver.h"

// 飞镖数据存储相关定义
//最大飞镖数据池大小?待确认
#define MAX_DART_DATAPOOL_SIZE 16

//飞镖数据结构体,用于选择目标是基地还是前哨站
typedef enum __DartAimEnumdef
{
  Outpost = 0,
  Base = 1
}DartAimEnumdef;

//飞镖数据结构体,用于存储飞镖目标数据
typedef struct __DartDataStructdef
{
  double Ignitergoal[2]; //扳机目标位置
  double YawCorrectionAngle[2];//偏航修正角
}DartDataStructdef;

/**
 * @brief 发射主控任务
 * @parma None
 * @return None
 */
enum Missle_State_t
{
	DEINIT,
	WAIT_ACT,
	PULL,
	BACK,
	WAIT_SHOOT
};

enum
{
  R = 0,
  L = 1
};

enum yaw_control_state_e
{
    MANUAL_AIM = 0,
    VISION_AIM = 1,
    CORRECT_AIM = 2,
    disable_motor =3,
    YAW_CALIBRATING =4
};

// 定义一个本地结构体，只保存我们需要的数据
struct DR16_Snapshot_t {
    LinkageStatus_Typedef Status;
    SW_Status_Typedef S1;
    SW_Status_Typedef S2;
    float RX_Norm;
    float RY_Norm;
    float LX_Norm;
    float LY_Norm;
};

/* 状态机枚举定义建议 (在 internal.h) */
enum LaunchState_e {
    STATE_OFFLINE,      // 离线/失能
    STATE_DEINIT,       // 正在初始化/归位
    STATE_WAIT_CMD,     // 等待发射指令 (滑块在缓冲区，随时可发)
    STATE_PULLING,      // 正在下拉蓄力
    STATE_LATCHING,     // 到底部，等待挂机稳定 (User code: PULL delay)
    STATE_RETURNING,    // 正在回升到缓冲区
    STATE_FIRING,       // 正在执行发射动作
    STATE_RELOADING,    // (预留) 正在执行自动装填动作
    STATE_COOLDOWN      // 发射后冷却
};


// --- 1. 枚举定义 (Enums) ---
typedef enum {
    SYS_OFFLINE = 0,    // 离线/故障
    SYS_CHECKING,       // 自检中
    SYS_CALIBRATING,    // 归零/校准中
    SYS_STANDBY,        // 待机/手动
    SYS_AUTO_PREP,      // 自动发射准备 (回缓冲区)
    SYS_AUTO_FIRE       // 自动发射进行中
} System_State_e;

typedef enum {
    TARGET_BASE = 0,
    TARGET_OUTPOST
} Target_Type_e;

typedef enum {
    BURST_2 = 2,
    BURST_4 = 4
} Burst_Mode_e;

// --- 2. 交互结构体 ---

// [指令 Cmd]: 任务层根据遥控器算出来的“意图”
typedef struct {
    bool sys_enable;        // 系统使能
    bool auto_mode;         // 自动模式开关
    bool manual_override;   // 手动接管开关
    bool skip_check;        // 跳过自检

    bool fire_command;      // 发射指令 (S2 或 视觉触发)
    bool calibration_req;   // 请求校准
    
    Target_Type_e target;   // 目标类型
    Burst_Mode_e  burst_num;// 连发数

    float manual_yaw_inc;   // 手动 Yaw 增量
    float manual_pitch_inc; // 手动 丝杆 增量
} Robot_Cmd_t;

// [监控 Monitor]: 硬件反馈上来的“健康状况”
typedef struct {
    bool rc_connected;      // 遥控器连接
    struct {
        bool limit_sw_ok;   // 限位开关自检通过
        bool motors_ok;     // 电机自检通过
        bool all_passed;    // 总通过
    } Check;
    struct {
        bool is_calibrated; // 系统是否已完全校准
    } Status;
} Robot_Monitor_t;

// [反馈 Feedback]: 运行中的实时数据
typedef struct {
    System_State_e current_state; // 当前主状态
    yaw_control_state_e yaw_control_state;//yaw轴控制状态
    uint8_t dart_count;           // 已发射计数
    // 这里不再放电机角度，因为电机角度归类管理，任务层只关心结果
} Robot_Feedback_t;

// --- 3. 全局数据包 ---
typedef struct {
    Robot_Cmd_t      Cmd;
    Robot_Monitor_t  Flag;
    Robot_Feedback_t Status;
} Robot_Ctrl_t;


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

#pragma pack(1)
    struct VisionSendData_t
    {
        uint8_t head = 0x45;
        float current_yaw;
        float base_yaw;
        uint8_t mode = 3;
        uint8_t tracker_bit = 0;
        uint8_t calibration_state = 0; // 标定
        uint8_t end = 0x55;
    };
#pragma pack()


extern DartAimEnumdef HitTarget;                            // 打击目标

extern Launcher_Driver Launcher; // 发射驱动类

extern Missle_YawController_Classdef Yawer; // yaw控制类

extern abstractMotor<Motor_GM6020> loadermotor; // 装填电机抽象类


extern uint32_t vision_last_recv_time ; // 视觉最后接收时间
extern VisionRecvData_t vision_recv_pack;
extern VisionSendData_t vision_send_pack;

extern Robot_Ctrl_t Robot; 
/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/

