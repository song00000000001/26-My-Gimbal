#pragma once

#include "robot_types.h"
#include "Yaw_control.h"
#include "launcher_driver.h"
#include "openlog.h"

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
    SYS_DEBUG,           // 调试状态
    SYS_CHECKING,       // 自检中
    SYS_CHECKED,        // 自检完成,用于延时，防止过快进入校准模式导致校准异常
    SYS_CALIBRATING,    // 归零/校准中
	SYS_CALIBRATED,      // 校准完成,归位
    SYS_STANDBY,        // 待机/手动
    SYS_AUTO_PREP,      // 自动发射准备 (回缓冲区)
    SYS_AUTO_FIRE       // 自动发射进行中
} System_State_e;


// [指令 Cmd]: 任务层根据遥控器算出来的“意图”
typedef struct {
    bool sys_enable;        // 系统使能
    bool autofire_enable;   // 自动发射模式标志位
    bool skip_check;        // 跳过自检
    bool fire_command;      // 发射指令 
    /*todo
    song
    后期和视觉联动可以使用该标志位
    并且增加超时处理。
    */

} Robot_Cmd_t;

// [监控 Monitor]: 硬件反馈上来的“状况”
typedef struct {
    struct {
        bool limit_sw_ok;   // 限位开关自检通过
        //bool motors_ok;     // 电机自检通过
        //bool all_passed;    // 总通过
    } Check;
    struct {
        bool rc_connected;      // 遥控器连接
        bool is_calibrated; // 系统是否已完全校准
        bool stop_continus_fire;    // 停止连发标志位
        bool tool_panel_connected; //调参板连接状态
        bool vision_connected; //视觉连接状态
    } Status;
} Robot_Monitor_t;

//[反馈 Feedback]: 运行中的实时数据
typedef struct {
    System_State_e current_state; // 当前主状态
    yaw_control_state_e yaw_control_state;//yaw轴控制状态
    uint8_t dart_count;           // 已发射计数

} Robot_Feedback_t;

//数据包
typedef struct {
    Robot_Cmd_t      Cmd;
    Robot_Monitor_t  Flag;
    Robot_Feedback_t Status;
} Robot_Ctrl_t;


//调试数据结构体
typedef struct {
    // 标志位
    uint8_t enable_debug_mode; // 在watch窗口改为true以进入调试模式(配合遥控器)
    //电机状态
    Control_Mode_e debug_mode_deliver[2]; // 左右滑块的独立模式
    Control_Mode_e debug_mode_igniter;    // 丝杆模式
} Debug_Data_t;

// 校准速度结构体
typedef struct {
	float yaw_calibration_speed;
	float deliver_calibration_speed;
	float igniter_calibration_speed;
}calibration_speed_t;


//调参板数据结构
//用于存储调参板传来的飞镖目标数据
typedef struct __DartDataStructdef
{
  double Ignitergoal[2]; //扳机目标位置
  double YawCorrectionAngle[2];//偏航修正角
}DartDataStructdef;

//用于选择目标是基地还是前哨站
typedef enum __DartAimEnumdef
{
  Outpost = 0,
  Base = 1
}DartAimEnumdef;


//定义舵机测试结构体，方便调节测试舵机行程
typedef struct 
{
    uint16_t igniter_ccr_unlock;
    uint16_t igniter_ccr_lock;
    uint16_t loader1_ccr_up;
    uint16_t loader1_ccr_down;
    uint16_t loader2_ccr_up;
    uint16_t loader2_ccr_down;
    uint16_t transfomer_ccr_lock;
    uint16_t transfomer_ccr_unlock;
}servo_ccr_debug;

//定义通信协议状态结构体，包括时间窗口内收到的包的情况，是否连接等
typedef struct 
{
    bool connected;
    uint16_t rx_count;
    uint16_t rx_max_count;
    float rx_rate;
    float rx_rate_threshold;
}protocol_status_t;

extern protocol_status_t Protocol_Status[4]; //4个电机的通信状态
extern Launcher_Driver Launcher; 
extern Missle_YawController_Classdef Yawer; 
extern Robot_Ctrl_t Robot; 
extern Debug_Data_t Debugger; 
extern calibration_speed_t calibration_speed;
extern DartDataStructdef DartsData[];   
extern DartAimEnumdef HitTarget;       
extern uint8_t DartDataSlot[];          
extern DR16_Snapshot_t DR16_Snap; 
extern uint32_t vision_last_recv_time ; 
extern VisionRecvData_t vision_recv_pack;
extern VisionSendData_t vision_send_pack;
extern servo_ccr_debug servo_ccr;
extern openlog_classdef<16> OpenLog;



/*
1. 写入内容到当前缓冲
2. 提交当前缓冲，让后台任务去发送
*/
#define ROBOT_LOG(level, format, ...) do { \
    OpenLog.record("[%s] " format "\r\n", level, ##__VA_ARGS__); \
    OpenLog.push_buff(); \
} while(0)

#define LOG_INFO(format, ...)  ROBOT_LOG("INFO", format, ##__VA_ARGS__)
#define LOG_WARN(format, ...)  ROBOT_LOG("WARN", format, ##__VA_ARGS__)
#define LOG_ERROR(format, ...) ROBOT_LOG("ERROR", format, ##__VA_ARGS__)

#ifdef __cplusplus
}
#endif