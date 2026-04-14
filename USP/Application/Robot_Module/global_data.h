#pragma once

#include "internal.h"
#include "motor_m0603A_driver.h"
#include "my_pid.h"
#include "motor_speed_ctrl.h"

#ifdef __cplusplus
extern "C" {
#endif

enum MotorIndex_e {
    PITCH = 0,
    YAW   = 1,
    MOTOR_COUNT = 2
};

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
    MotorIndex_e motor_index; ///< 选择要调试的电机，0表示PITCH，1表示YAW
    MotorMode motor_mode; ///< 电机模式，OPEN_LOOP=0, CURRENT_LOOP=1, SPEED_LOOP=2, POSITION_LOOP=3
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


extern BenMoMotor g_motors[MOTOR_COUNT];

extern MyPid_Struct g_pid[MOTOR_COUNT];
extern MyPid_Debug_Struct g_pid_debug[MOTOR_COUNT]; 

extern float g_imu_angle_deg[2];
extern float g_imu_gyro_dps[2];

extern BenMoMotor* motor_observer; 
extern MyPid_Struct *pid_observer;
extern MyPid_Debug_Struct *pid_debug_observer;
extern MotorSpeedCtrl *spd_ctrl_observer;
extern MotorSpeedCtrlParam *spd_ctrl_param_observer;
#pragma pack()

#ifdef __cplusplus
}
#endif