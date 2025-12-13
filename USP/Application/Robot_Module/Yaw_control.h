#pragma once

#ifdef __cplusplus

#include "SRML.h"
#include "robot_types.h"

enum yaw_control_state_e
{
    MANUAL_AIM = 0,
    VISION_AIM,
    CORRECT_AIM,
    YAW_CALIBRATING 
};

class Missle_YawController_Classdef
{
private:
  float MAX_YAW_ANGLE, MIN_YAW_ANGLE;
 
  // 堵转计时器
  uint32_t stall_timer_yaw;

public:

uint8_t Yaw_Init_flag = 0;
myPID PID_Yaw_Angle;
myPID PID_Yaw_Speed;
  Control_Mode_e mode_YAW;
  float yaw_target = 0;
  float yaw_correct_angle=0;        //yaw轴修正角
  abstractMotor<Motor_GM6020> YawMotor;
  inline bool is_Yaw_Init() { return (Yaw_Init_flag == 2); }

  Missle_YawController_Classdef(uint8_t _ID_YAW);
  void calibration();
  void update(float _yaw_target);
  void adjust();
  void disable();
  void yaw_out_motor_speed();
  void yaw_state_machine(yaw_control_state_e yaw_state,float LX,float LY);
  bool yaw_stall_check(float limit_output, float threhold_rpm, uint32_t time_ms);
  bool isMotorAngleReached(float threshold);
};

#endif