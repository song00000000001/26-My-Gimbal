/**
  ******************************************************************************
  * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
  * @file    YawController_Classdef.h
  * @author  lpk
  * @brief   Header file 
  ******************************************************************************
  * @attention
  * 
  * if you had modified this file, please make sure your code does not have many
  * bugs, update the version Number, write dowm your name and the date, the most
  * important is make sure the users will have clear and definite understanding 
  * through your new brief.
  *
  * <h2><center>&copy; Copyright (c) 2019 - ~, SCUT-RobotLab Development Team.
  * All rights reserved.</center></h2>
  ******************************************************************************
  */
#pragma once

#ifdef __cplusplus

/* Includes ------------------------------------------------------------------*/
#include "SRML.h"
/* Private macros ------------------------------------------------------------*/

/* Private type --------------------------------------------------------------*/

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
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

};
/* Exported function declarations --------------------------------------------*/

#endif