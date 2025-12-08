/**
  * @file   launcher_driver.h
  * @brief  发射机构底层驱动 (无业务逻辑)
  */
#ifndef _LAUNCHER_DRIVER_H_
#define _LAUNCHER_DRIVER_H_

#include "SRML.h"

// 定义内部控制模式
typedef enum {
    MODE_SPEED=0,       // 速度环
    MODE_ANGLE,      // 角度环
    MODE_ERROR      // 失能
} Control_Mode_e;

class Launcher_Driver
{
private:
    /* --- 1. 硬件对象 (Private: 外部不可见) --- */

    
    // 限位开关读取函数指针
    GPIO_PinState (*read_switch_L)(void);
    GPIO_PinState (*read_switch_R)(void);
    GPIO_PinState (*read_switch_Ign)(void);

    /* --- 2. 算法对象 (Private) --- */
    myPID pid_deliver_spd[2];
    myPID pid_deliver_pos[2];
    myPID pid_deliver_sync;    // 双电机同步PID
    myPID pid_igniter_spd;
    myPID pid_igniter_pos;

    /* --- 3. 内部状态 (Private) --- */
    Control_Mode_e mode_deliver[2]; // 左右滑块的独立模式
    Control_Mode_e mode_igniter;    // 丝杆模式
    
    
    bool is_deliver_homed[2];       // 是否已完成归零
    bool is_igniter_homed;
    

  

public:

abstractMotor<Motor_C620> DeliverMotor[2]; // [0]=Left, [1]=Right
abstractMotor<Motor_C610> IgniterMotor;
// 记录哪几个开关已经检测过了 (Bitmask)
uint8_t check_progress; 
float target_deliver_angle;     // 滑块目标角度 (位置模式用)
    float target_igniter_angle;     // 丝杆目标角度

    /* --- 构造函数 --- */
    Launcher_Driver(uint8_t id_l, uint8_t id_r, uint8_t id_ign);
    
    /* --- 2. 动作接口 (Command) --- */
    // 启动归零程序 (将模式切为 HOMING)
    void start_calibration();
    
    // 强制停止/失能
    void stop_yaw_motor();
    void stop_deliver_motor();
    void stop_igniter_motor();
    void stop_all_motor();

    // 舵机动作 (直接操作硬件，简单封装)
    void fire_unlock(); 
    void fire_lock();

    /* --- 3. 核心运行 (Execute) --- */
    // 放在 1ms 任务或定时器中调用
    void run_1ms();

    /* --- 4. 状态查询 (Feedback) --- */
    // 获取当前状态，用于任务层判断是否可以下一步
    bool is_calibrated();          // 全系统是否已校准
    bool is_deliver_at_target();   // 滑块是否到位
    bool is_igniter_at_target();   // 丝杆是否到位

    void key_check();

    // 检查限位开关并处理归零逻辑
    void check_calibration_logic(); 

    void out_all_motor_speed();
};

#endif