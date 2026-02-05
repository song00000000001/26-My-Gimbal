#pragma once

#include "SRML.h"
#include "robot_types.h"

typedef struct motor_angle_limit_t{
    float lower_limit;
    float upper_limit;
} motor_angle_limit_t;

class motor_ctrl_driver
{
private:

public:
    // PID 对象
    myPID mymotor_pid_spd,mymotor_pid_pos;    

    //抽象电机对象
    abstractMotor<Motor_C620> mymotor;
    Control_Mode_e mymotor_mode;    
    float target_motor_angle;    
    float threshold_motor_at_target=1.0f; // 角度环目标到达阈值
    motor_angle_limit_t mymotor_limit;
    // 根据电机模式（角度环，速度环，失能）调用 PID 计算
    motor_ctrl_driver(uint8_t id);
    void adjust();

    // 输出所有电机控制电流
    void motor_output(bool enable);

    // 是否到达目标位置
    bool is_motor_at_target();
};
