#pragma once

#include "robot_types.h"
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

extern uint32_t vision_last_recv_time ; // 视觉最后接收时间
extern VisionRecvData_t vision_recv_pack;
extern VisionSendData_t vision_send_pack;


//todo
//dr16快照结构体
struct DR16_Snapshot_t {
    LinkageStatus_Typedef Status;
    SW_Status_Typedef S1;
    SW_Status_Typedef S2;
    float RX_Norm;
    float RY_Norm;
    float LX_Norm;
    float LY_Norm;
};
///todo
typedef enum {
    BURST_2 = 2,
    BURST_4 = 4
} Burst_Mode_e;

//主状态枚举
typedef enum {
    SYS_OFFLINE = 0,    // 离线
    SYS_ERROR,           // 错误状态,一般是运行中电机失控装限位开关，或者触发堵转保护
    SYS_DEBUG,           // 调试状态
    SYS_CHECKING,       // 自检中
    SYS_CALIBRATING,    // 归零/校准中
    SYS_STANDBY,        // 待机/手动
    SYS_AUTO_PREP,      // 自动发射准备 (回缓冲区)
    SYS_AUTO_FIRE       // 自动发射进行中
} System_State_e;

// --- 2. 交互结构体 ---

// [指令 Cmd]: 任务层根据遥控器算出来的“意图”
typedef struct {
    bool sys_enable;        // 系统使能
    bool auto_mode;         // 自动模式开关
    bool manual_override;   // 手动微调开关
    bool skip_check;        // 跳过自检
    bool fire_command;      // 发射指令 (todo视觉触发)

} Robot_Cmd_t;

// [监控 Monitor]: 硬件反馈上来的“健康状况”
typedef struct {
    struct {
        bool limit_sw_ok;   // 限位开关自检通过
        //bool motors_ok;     // 电机自检通过
        //bool all_passed;    // 总通过
    } Check;
    struct {
        bool rc_connected;      // 遥控器连接
        bool is_calibrated; // 系统是否已完全校准
    } Status;
} Robot_Monitor_t;

// [反馈 Feedback]: 运行中的实时数据
typedef struct {
    System_State_e current_state; // 当前主状态
    yaw_control_state_e yaw_control_state;//yaw轴控制状态
    
    uint8_t dart_count;           // 已发射计数

} Robot_Feedback_t;

// --- 3. 全局数据包 ---
typedef struct {
    Robot_Cmd_t      Cmd;
    Robot_Monitor_t  Flag;
    Robot_Feedback_t Status;
} Robot_Ctrl_t;

extern Launcher_Driver Launcher; // 发射驱动类
extern Missle_YawController_Classdef Yawer; // yaw控制类
extern Robot_Ctrl_t Robot; 

typedef struct {
    float limit_output;
    float threhold_rpm;
    uint32_t time_ms;
} stall_params_t;

// 2. 新增调试数据结构体
typedef struct {
    // 标志位
    bool enable_debug_mode; // 在watch窗口改为true以进入调试模式(配合遥控器)
    //电机状态
    Control_Mode_e debug_mode_deliver[2]; // 左右滑块的独立模式
    Control_Mode_e debug_mode_igniter;    // 丝杆模式
    //堵转参数
    stall_params_t stall_params_deliver;
    stall_params_t stall_params_igniter;
    stall_params_t stall_params_yaw;
} Debug_Data_t;

extern Debug_Data_t Debugger; // 声明全局变量


enum Missle_State_t
{
	DEINIT,
	WAIT_ACT,
	PULL,
	BACK,
	WAIT_SHOOT
};

// 校准速度结构体
typedef struct {
	float yaw_calibration_speed;
	float deliver_calibration_speed;
	float igniter_calibration_speed;
}calibration_speed_t;

extern calibration_speed_t calibration_speed;


