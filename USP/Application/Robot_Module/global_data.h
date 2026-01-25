#pragma once

#include "robot_types.h"
#include "Yaw_control.h"
#include "launcher_driver.h"
#include "openlog.h"
#include "internal.h"

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
#if enum_X_Macros_disable
typedef enum {
    SYS_DISCONNECTED = 0,    // 离线
    SYS_MANUAL_TEST_KEY,       // 自检中
    SYS_START_UP,        // 自检完成,用于延时，防止过快进入校准模式导致校准异常
    SYS_HOMING,    // 归零/校准中
	SYS_READY,      // 校准完成,归位
    SYS_AUTOFIRE_SUSPEND,        // 待机/手动
    SYS_AUTO_PREP,      // 自动发射准备 (回缓冲区)
    SYS_AUTO_FIRE       // 自动发射进行中
} System_State_e;

#else
/*
为了解决“手动维护两个列表”的问题，使用 X-Macros。只需定义一次列表，编译器会自动生成 enum 定义和对应的“名称转换”函数。
*/
//1. 定义 X-列表：

#define SYSTEM_STATE_LIST(X) \
    X(SYS_DISCONNECTED)\
    X(SYS_MANUAL_TEST_KEY)\
    X(SYS_HOMING)\
	X(SYS_READY)\
    X(SYS_AUTOFIRE_SUSPEND)\
    X(SYS_AUTO_PREP)\
    X(SYS_AUTO_FIRE)

// 自动生成枚举定义
enum System_State_e {
    #define AS_ENUM(name) name,
    SYSTEM_STATE_LIST(AS_ENUM)
    #undef AS_ENUM
};

//2. 自动生成转换函数：
inline const char* System_State_To_Str(System_State_e state) {
    switch(state) {
        #define AS_CASE(name) case name: return #name;
        SYSTEM_STATE_LIST(AS_CASE)
        #undef AS_CASE
        default: return "UNKNOWN";
    }
}
//3. 在日志中使用：
//LOG_INFO("State changed to: %s", System_State_To_Str(Robot.Status.current_state));

//增加新状态时，只需在 SYSTEM_STATE_LIST 中添加一行，枚举和字符串会自动同步。
#endif

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
        uint8_t is_calibrated; // 系统是否已完全校准
        bool stop_continus_fire;    // 停止连发标志位
        bool tool_panel_connected; //调参板连接状态
        bool vision_connected; //视觉连接状态
        bool emergency_override; //紧急预案,改为自动触发发射暂停,并且切换RY手动接管滑块电机角度环位置控制。
        bool safely_abort_fire; //安全中止发射标志位
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
    float debug_loader_pos; //调试滑块位置
    uint8_t debug_fire_type; //调整发射类型，0为连发一二三四，1为单发第一发，2为单发第二发，3为单发第三发。

    bool is_loader_simulating;    // 模拟标志位
    float simulated_loader_pos; // 模拟滑块位置
    bool four_dart_four_params_enable; //四发四参功能启用标志位
    float dual_loader_mechanical_error_correction; //双滑块机械装配误差校准修正
	float deliver_sync_threshold; //滑块同步误差阈值
    bool initial_calibration_flag; //初始化校准标志位，用于跳过遥控失联校准流程。
    float emegency_deliver_ctrl_speed;
    float deliver_speed_limit; //滑块速度环限幅
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

//定义发射流程延时参数结构体
//用于调整发射流程中的各个延时参数
typedef struct 
{
    uint16_t put_delay;
    uint16_t before_fire_delay;
    uint16_t after_fire_delay;
    uint16_t relapse_delay;
    uint16_t loader_up_delay;   
    uint16_t wait_for_aim_delay;
    uint16_t deliver_pulldown_timeout;
}fire_sequence_delay_params_t;


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
extern fire_sequence_delay_params_t fire_sequence_delay_params;

/*
1. 写入内容到当前缓冲
2. 提交当前缓冲，让后台任务去发送
信号量保护，只等待1个RTOS节拍，防止主任务阻塞
由于每个任务都可能调用日志，而且存在相互控制的情况，很可能发生死锁，考虑到日志系统的实时性要求不高，决定去掉信号量保护，减少开销。
*/
#if 0
#define ROBOT_LOG(level, format, ...) do { \
    if (xSemaphoreTake(OpenLog_mutex, 0) == pdTRUE) { \
        OpenLog.record("[%s] " format "\r\n", level, ##__VA_ARGS__); \
        OpenLog.push_buff(); \
        xSemaphoreGive(OpenLog_mutex); \
    } \
} while(0)
#else
//考虑进行宏优化，即筛去重复信息，或者重复就写到一行里面，不触发push操作。
//但日志系统本意是为了调试方便，不建议过度优化，防止遗漏重要信息。
//写入日志时务必注意不要频繁调用，尤其是在高频率循环或中断中，以免日志系统成为性能瓶颈。
#define ROBOT_LOG(level, format, ...) do { \
    OpenLog.record("[%s] " format "\r\n", level, ##__VA_ARGS__); \
    OpenLog.push_buff(); \
} while(0)
#endif

#define LOG_INFO(format, ...)  ROBOT_LOG("INFO", format, ##__VA_ARGS__)
#define LOG_WARN(format, ...)  ROBOT_LOG("WARN", format, ##__VA_ARGS__)
#define LOG_ERROR(format, ...) ROBOT_LOG("ERROR", format, ##__VA_ARGS__)

// 将 8 位数据的后 5 位转换为二进制字符串的宏 (例如: "01011")
#define BIN5_FMT "%c%c%c%c%c"
#define BIN5_VAL(val) \
    ((val) & 0x10 ? '1' : '0'), \
    ((val) & 0x08 ? '1' : '0'), \
    ((val) & 0x04 ? '1' : '0'), \
    ((val) & 0x02 ? '1' : '0'), \
    ((val) & 0x01 ? '1' : '0')

#ifdef __cplusplus
}
#endif