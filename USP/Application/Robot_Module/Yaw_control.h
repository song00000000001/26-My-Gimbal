#pragma once

#ifdef __cplusplus

#include "SRML.h"

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
  myPID PID_Yaw_Angle;
  myPID PID_Yaw_Speed;
  uint8_t Yaw_Init_flag = 0;
public:

  abstractMotor<Motor_GM6020> YawMotor;
  inline bool is_Yaw_Init() { return (Yaw_Init_flag == 2); }

  Missle_YawController_Classdef(uint8_t _ID_YAW);
  void calibration();
  void update(float _yaw_target);
  void adjust();
  void disable();
  void yaw_out_motor_speed();
  void yaw_state_machine(yaw_control_state_e yaw_state,float yaw_manual_target);
};

#endif