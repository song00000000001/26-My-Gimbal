/**
  * @file   launcher_driver.h
  * @brief  发射机构底层驱动 (无业务逻辑)
  */
#ifndef _LAUNCHER_DRIVER_H_
#define _LAUNCHER_DRIVER_H_

#include "SRML.h"
#include "robot_types.h"


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

class Launcher_Driver
{
private:
    /* --- 1. 硬件对象 (Private: 外部不可见) --- */
    // 堵转计时器
    uint32_t stall_timer_deliver;
    uint32_t stall_timer_igniter;
    
    // 限位开关读取函数指针
    GPIO_PinState (*read_switch_L)(void);
    GPIO_PinState (*read_switch_R)(void);
    GPIO_PinState (*read_switch_Ign)(void);

    /* --- 2. 算法对象 (Private) --- */
   
 
public:
bool is_deliver_homed[2];       // 是否已完成归零
bool is_igniter_homed;


    myPID pid_deliver_spd[2];
    myPID pid_deliver_pos[2];
    myPID pid_deliver_sync;    // 双电机同步PID
    myPID pid_igniter_spd;
    myPID pid_igniter_pos;
    // 发射子状态机
    Fire_State_e fire_state = FIRE_IDLE;

    //电机状态
    Control_Mode_e mode_deliver[2]; // 左右滑块的独立模式
    Control_Mode_e mode_igniter;    // 丝杆模式

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
    void adjust();

    /* --- 4. 状态查询 (Feedback) --- */
    // 获取当前状态，用于任务层判断是否可以下一步
    bool is_calibrated();          // 全系统是否已校准
    bool is_deliver_at_target();   // 滑块是否到位
    bool is_igniter_at_target();   // 丝杆是否到位

    void key_check();

    // 检查限位开关并处理归零逻辑
    void check_calibration_logic(); 
    // 输出所有电机当前速度
    void out_all_motor_speed();
    // 发射子状态机
    void Run_Firing_Sequence();
    // 堵转检测 (无电流计版)
    bool check_deliver_stall(float limit_output,float threhold_rpm, uint32_t time_ms);
    bool check_igniter_stall(float limit_output,float threhold_rpm, uint32_t time_ms);

};

#endif