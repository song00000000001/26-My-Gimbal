/**
  * @file   launcher_driver.h
  * @brief  发射机构底层驱动 (无业务逻辑)
  */
#ifndef _LAUNCHER_DRIVER_H_
#define _LAUNCHER_DRIVER_H_

#include "SRML.h"
#include "robot_types.h"

#if 0
/* --- 4. 发射流程子状态机 --- */
// 定义子状态
typedef enum {
    FIRE_IDLE=0,
    FIRE_PULL_LOAD,     //下拉到装填位置
    FIRE_WAITLOAD,      //等待装填
    FIRE_PULL_BOTTOM,    // 下拉到底
    FIRE_LATCHING,   // 挂机 (等待装填归位)
    FIRE_RETURNING,  // 回升(回到缓存区)
    FIRE_TRANSFORE,         //动作仓储区舵机,转移新镖
    FIRE_TRANSFORE_BACK,      //动作仓储区舵机,卡镖舵机回正，卡住镖
    FIRE_SHOOTING,   // 开火 (舵机)
} Fire_State_e;
#else
/* --- 4. 发射流程子状态机 --- */
typedef enum {
    FIRE_IDLE = 0,          // 闲置/决策状态
	FIRE_IGNITER_DELAY,
    //第一发
    FIRE_CALIBRATION_1,    // 校准滑块电机
    FIRE_PULL_DOWN_1,       // 滑块下拉
    FIRE_WAIT_BOTTOM_1,     // 底部等待
    FIRE_RETURN_UP_1,       // 滑块回缓冲区
    FIRE_WAIT_UP_1,         // 缓冲区等待
    FIRE_SHOOTING_1,        // 射击

    //第二发
    FIRE_CALIBRATION_2,    // 校准滑块电机
    FIRE_PULL_DOWN_2,       // 滑块下拉
    FIRE_WAIT_BOTTOM_2,     // 底部等待
    FIRE_RETURN_UP_2,       // 滑块回缓冲区
    FIRE_WAIT_UP_2,         // 缓冲区等待
    FIRE_RELOAD_LOWER_2,    // 升降机下降
    FIRE_SHOOTING_2,        // 射击

    //第三发
    FIRE_CALIBRATION_3,    // 校准滑块电机
    FIRE_RELOAD_LIFT_3,      // 升降机上升
    FIRE_RELOAD_RELEASE_3,   // 卡镖释放
    FIRE_PULL_DOWN_3,        // 滑块下拉
    FIRE_WAIT_BOTTOM_3,      // 底部等待
    FIRE_RETURN_UP_3,        // 滑块回缓冲区
    FIRE_WAIT_UP_3,          // 缓冲区等待
    FIRE_RELOAD_LOWER_3,     // 升降机下降
    FIRE_SHOOTING_3,         // 射击

    //第四发
    FIRE_CALIBRATION_4,    // 校准滑块电机
    FIRE_RELOAD_LIFT_4,      // 升降机上升
    FIRE_RELOAD_RELEASE_4,   // 卡镖释放
    FIRE_PULL_DOWN_4,        // 滑块下拉
    FIRE_WAIT_BOTTOM_4,      // 底部等待
    FIRE_RETURN_UP_4,        // 滑块回缓冲区
    FIRE_WAIT_UP_4,          // 缓冲区等待
    FIRE_RELOAD_LOWER_4,     // 升降机下降
    FIRE_SHOOTING_4,         // 射击
          
} Fire_State_e;
#endif
class Launcher_Driver
{
private:
 
public:
    // 校准状态
    bool is_deliver_homed[2],is_igniter_homed;     
    
    uint32_t calibration_start_time; // 记录校准开始时间，用于超时检测

    // PID 对象
    myPID pid_deliver_spd[2],pid_deliver_pos[2];    
    myPID pid_deliver_sync;    // 双电机同步PID
    myPID pid_igniter_spd,pid_igniter_pos;
   
    // 发射子状态机
    Fire_State_e fire_state = FIRE_IDLE;

    //电机状态
    Control_Mode_e mode_deliver[2]; // 左右滑块的独立模式
    Control_Mode_e mode_igniter;    // 丝杆模式

    //抽象电机对象
    abstractMotor<Motor_C620> DeliverMotor[2];  // [0]=Left, [1]=Right
    abstractMotor<Motor_C620> IgniterMotor;

    // 记录哪几个开关已经检测过了 (Bitmask)
    uint8_t check_progress; 
    // 目标位置，debug时可以
    float target_deliver_angle;     // 滑块目标角度 (位置模式用)
    float target_igniter_angle;     // 丝杆目标角度

    Launcher_Driver(uint8_t id_l, uint8_t id_r, uint8_t id_ign);
    
    // 启动校准(归零)程序
    void start_calibration();
    
    // 强制停止/失能
    void stop_deliver_motor();
    void stop_igniter_motor();
    void stop_all_motor();

    // 根据电机模式（角度环，速度环，失能）调用 PID 计算
    void adjust();

    // 获取当前状态，用于任务层判断是否可以下一步
    bool is_calibrated();          // 全系统是否已校准
    bool is_deliver_at_target(float threshold);   // 滑块是否到位
    bool is_igniter_at_target(float threshold);   // 丝杆是否到位
    
    // 自检状态检查按键状态
    void key_check();

    // 检查限位开关并处理归零逻辑
    void check_calibration_logic(); 
    // 输出所有电机当前速度
    void out_all_motor_speed();
    // 发射子状态机
    void Run_Firing_Sequence();

    //舵机PWM测试
    void servo_pwm_test_lock_up();
    void servo_pwm_test_unlock_down();

    // 发射状态机中校准滑块电机
    void start_deliver_calibration();
};

#endif