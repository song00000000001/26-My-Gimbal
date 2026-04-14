#pragma once

#include "internal.h"
#include "motor_m0603A_driver.h"
#include "my_pid.h"
//#include "motor_ctrl_driver.h"
//#include "remote_ctrl_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

enum MotorIndex {
    PITCH = 0,
    YAW   = 1,
    MOTOR_COUNT = 2
};
extern BenMoMotor gimbal_motors[MOTOR_COUNT]; // 电机实例数组

typedef enum{
    debug_idle = 0,
    debug_mtvofa_monitor,
    debug_mpuvoda_monitor
}Debug_Mode_e;

//调试数据结构体
typedef struct {
    Debug_Mode_e enable_debug_mode; ///< 在watch窗口改为true以进入调试模式(配合遥控器)
    //uint8_t motor_accel_time; ///< 电机加速时间，单位ms/1rpm，0表示最快
    //bool motor_brake_enable; ///< 电机刹车使能，true表示启用刹车，false表示不刹车
    //uint8_t motor_mode; ///< 电机模式，0表示电流环模式，1表示速度环模式，2表示位置环模式
    MotorMode motor_mode; ///< 电机模式，OPEN_LOOP=0, CURRENT_LOOP=1, SPEED_LOOP=2, POSITION_LOOP=3
    bool angle_loop_enable; ///< 由于角度环串速度环需要单独调试，这里增加一个开关用于切换
    bool speed_loop_enable; ///< 增加速度环串电流环单独调试的开关
    float spd_target_rpm; ///< 用于单独调速度环时给定速度target，单位为RPM
    bool spd_feedback_source; ///< 用于切换速度环反馈数据源, false表示使用电机速度反馈，true表示使用IMU角速度数据
    bool system_enable; ///< 系统使能
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

extern MyPid gimbal_pid_pos[MOTOR_COUNT]; // PID控制器实例数组
extern MyPid gimbal_pid_spd[MOTOR_COUNT]; // PID控制器实例数组
extern MyPid gimbal_pid_cur[MOTOR_COUNT]; // PID控制器实例数组
extern MyPidParam gimbal_pid_param_pos[MOTOR_COUNT]; // PID参数实例数组
extern MyPidParam gimbal_pid_param_spd[MOTOR_COUNT]; // PID参数实例数组
extern MyPidParam gimbal_pid_param_cur[MOTOR_COUNT]; // PID参数实例数组
extern float hold_angle_deg[2]; // 固定的目标位置，单位为度
extern float imu_angle_deg[2]; // 来自IMU的当前角度反馈，单位为度
extern float imu_gyro_dps[2]; // 来自IMU的角速度反馈，单位为度每秒
#pragma pack()

#ifdef __cplusplus
}
#endif