#pragma once

#ifdef __cplusplus

#include "SRML.h"
#include "robot_types.h"

enum yaw_control_state_e
{
    MANUAL_AIM = 0,
    VISION_AIM,
    CORRECT_AIM,
    YAW_CALIBRATING,
    DISABLE_MOTOR 
};

class Missle_YawController_Classdef
{
private:
  float MAX_YAW_ANGLE, MIN_YAW_ANGLE;
public:
    // yaw轴初始化标志，0未初始化，1初始化中，2初始化完成
    uint8_t Yaw_Init_flag = 0;
    // pid对象
    myPID PID_Yaw_Angle ,PID_Yaw_Speed;
    //电机模式
    Control_Mode_e mode_YAW;
    //yaw_target
    float yaw_target = 0;
    //电机抽象对象
    abstractMotor<Motor_GM6020> YawMotor;

    // 构造函数
    Missle_YawController_Classdef(uint8_t _ID_YAW);
    // 校准函数
    void calibration();
    // 更新yaw轴目标角度，带限幅，虽然说任务里是直接修改target的，也可封装，我觉得没必要。
    void update(float _yaw_target);
    // 根据电机模式进行pid计算
    void adjust();
    // 失能函数
    void disable();
    // 输出电机速度
    void yaw_out_motor_speed();
    //yaw轴子状态机,包含状态如下,同时会对行程电机进行控制
    /*
        manual_aim:手动瞄准
        vision_aim:视觉瞄准
        correct_aim调参板瞄准
        disable_motor:失能电机
        yaw_calibrating:校准模式
    */
    
    void yaw_state_machine(yaw_control_state_e *yaw_state,float RC_X,float RC_Y);
    // 判断yaw轴是否到达目标角度
    bool isMotorAngleReached(float threshold);
        
    // 判断yaw轴是否初始化完成
    inline bool is_Yaw_Init() { return (Yaw_Init_flag == 2); }
};

#endif