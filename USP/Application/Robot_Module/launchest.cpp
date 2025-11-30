/**
  ******************************************************************************
  * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
  * @file   launchest.cpp
  * @author lpk
  * @brief  发射控制代码
  * @date    2025/03/9 
  * @version 1.0
  * @par Change Log:
  * <table>
  * <tr><th>Date <th>Version <th>Author <th>Description
  * <tr><td>2024-03-31 <td> 1.0 <td>S.B. <td>Creator
  * </table>
  *
  ==============================================================================
                               How to use this Lib
  ==============================================================================
    @note
      -#
      -#
    @warning
      -#
      -#
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

/* Includes ------------------------------------------------------------------*/
#include "launchest.h"
#include "internal.h"
#include "global_data.h"
#include "robot_config.h"

/* Private define ------------------------------------------------------------*/
#define READ_IGNITERSWITCH HAL_GPIO_ReadPin(SW_IGNITER_GPIO_Port, SW_IGNITER_Pin)
/* Private variables ---------------------------------------------------------*/

/* Private type --------------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/
Launch_Classdef::Launch_Classdef(uint8_t _ID_DELIVER_R, uint8_t _ID_DELIVER_L, uint8_t _ID_IGNITER_R)
    : IgniterMotor(_ID_IGNITER_R),
      DeliverMotor{_ID_DELIVER_R, _ID_DELIVER_L}
{
    READ_DELIVERSWITCH[L] = []() -> GPIO_PinState
    { return HAL_GPIO_ReadPin(SW_DELIVER_L_GPIO_Port, SW_DELIVER_L_Pin); };

    READ_DELIVERSWITCH[R] = []() -> GPIO_PinState
    { return HAL_GPIO_ReadPin(SW_DELIVER_R_GPIO_Port, SW_DELIVER_R_Pin); };

    /* Polarity */
    DeliverMotor[L].Polarity = POLARITY_DELIVER_L;
    DeliverMotor[R].Polarity = POLARITY_DELIVER_R;
    IgniterMotor.Polarity = POLARITY_IGNITER;

    DeliverMotor[L].angle_unit_convert = (2 * PI * 18.62f) / (360 * 51);
    DeliverMotor[R].angle_unit_convert = (2 * PI * 18.62f) / (360 * 51);
    IgniterMotor.angle_unit_convert = 4 / (360.f * 36.f);//4是导程

    /* PIDinit */
    PID_Deliver_Diff.SetPIDParam(0.5f, 0.0f, 0.0f, 8000, 16000);
    PID_Deliver_Speed[R].SetPIDParam(20.0f, 2.0f, 0.0f, 8000, 16380);
    PID_Deliver_Speed[L].SetPIDParam(20.0f, 2.0f, 0.0f, 8000, 16380);
    PID_Deliver_Angle[R].SetPIDParam(800.f, 0.0, 0.0, 1000, 8000);
    PID_Deliver_Angle[L].SetPIDParam(800.f, 0.0, 0.0, 1000, 8000);
    PID_Igniter_Speed.SetPIDParam(15.0, 0.0, 0.0, 3000, 16000);
    PID_Igniter_Angle.SetPIDParam(3000.0, 0.0, 0.0, 3000, 7000);

    // PID_Deliver_Speed[R].Out_Max = PID_Deliver_Speed[L].Out_Max = 0;
}

void Launch_Classdef::Deliver_Init()
{
    for (int i = 0; i < 2; i++)
    {
        if(READ_DELIVERSWITCH[i]() == GPIO_PIN_RESET)
        {
            PID_Deliver_Angle[i].Target = -10;//到达复位的地点所需要的角度
            PID_Deliver_Speed->clean_intergral();
            DeliverMotor[i].baseAngle -= DeliverMotor[i].getMotorTotalAngle();//记录当前角度？？
            Deliver_Init_flag[i] = true;
            Pull_Ready_flag = false;
        }
        else
        {
            if(Deliver_Init_flag[i] == false)
            {
                PID_Deliver_Speed[i].Target = INIT_SPEED_DELIVER;
            }
            else
            {
                PID_Deliver_Angle[i].Target = -10;
            }
        }
    }
}

void Launch_Classdef::Igniter_Init()
{
    if (READ_IGNITERSWITCH == GPIO_PIN_RESET)
    {
        IgniterMotor.baseAngle -= IgniterMotor.getMotorTotalAngle();
        Igniter_Init_flag = true;
        PID_Igniter_Angle.Target = 3;
        PID_Igniter_Angle.clean_intergral();
    }
    else
    {
        if(Igniter_Init_flag == false)
        {
            PID_Igniter_Speed.Target = INIT_SPEED_IGNITER;
        }
        else
        {
            PID_Igniter_Angle.Target = 3;
        }
    }
}

void Launch_Classdef::Deliver_Pull()
{
    if (is_Deliver_Init() == 0 && Pull_Ready_flag == 0) // 即没有复位、也没有完成装填
    {
        Deliver_Init();
        return;
    }
    else if(Pull_Ready_flag == 1)   // 完成装填
    {
        return;
    }

    TargetAngle_Deliver = -640;//-485这个值会根据实车去修改
    for (int i = 0; i < 2; i++)
    {
        PID_Deliver_Angle[i].Target = TargetAngle_Deliver;
        PID_Deliver_Angle[i].Error = PID_Deliver_Angle[i].Target - PID_Deliver_Angle[i].Current;
    }

    if (std::abs(PID_Deliver_Angle[L].Error) < 1 && std::abs(PID_Deliver_Angle[R].Error) < 1)
    {
        Pull_Ready_flag = true;
        Deliver_Init_flag[R] = Deliver_Init_flag[L] = false;
    }

}

void Launch_Classdef::adjust()
{
    PID_Deliver_Diff.Target = 0;
    PID_Deliver_Diff.Current = DeliverMotor[R].getMotorTotalAngle() - DeliverMotor[L].getMotorTotalAngle();
    PID_Deliver_Diff.Adjust();
    if(is_Deliver_Init() == true)
    {
        PID_Deliver_Angle[R].Target += PID_Deliver_Diff.Out;
        PID_Deliver_Angle[L].Target -= PID_Deliver_Diff.Out;
    }
    else
    {
        PID_Deliver_Diff.clean_intergral();
    }

    for (int i = 0; i < 2; i++)
    {
        if(Deliver_Init_flag[i] == true)
        {
            PID_Deliver_Angle[i].Target = std_lib::constrain(PID_Deliver_Angle[i].Target, -640.f, 0.f);
            PID_Deliver_Angle[i].Current = DeliverMotor[i].getMotorTotalAngle();
            PID_Deliver_Angle[i].Adjust();
            PID_Deliver_Speed[i].Target = PID_Deliver_Angle[i].Out;
        }
        else
        {
            PID_Deliver_Angle[i].Target = PID_Deliver_Angle[i].Current = DeliverMotor[i].getMotorTotalAngle();
            PID_Deliver_Angle[i].Adjust();
        }
        PID_Deliver_Speed[i].Current = DeliverMotor[i].getMotorSpeed();
        PID_Deliver_Speed[i].Adjust();
        DeliverMotor[i].setMotorCurrentOut(PID_Deliver_Speed[i].Out);
    }
		
    if (Igniter_Init_flag == true)
    {
        PID_Igniter_Angle.Current = IgniterMotor.getMotorTotalAngle();
        PID_Igniter_Angle.Adjust();
        PID_Igniter_Speed.Target = PID_Igniter_Angle.Out;
    }
    else
    {
        PID_Igniter_Angle.Target = PID_Igniter_Angle.Current = IgniterMotor.getMotorTotalAngle();
        PID_Igniter_Angle.Adjust();
    }
    PID_Igniter_Speed.Current = IgniterMotor.getMotorSpeed();
    PID_Igniter_Speed.Adjust();
    IgniterMotor.setMotorCurrentOut(PID_Igniter_Speed.Out);
}

void Launch_Classdef::Igniter_Pos_Set(float _target_pos)
{
    PID_Igniter_Angle.Target = _target_pos;
}

void Launch_Classdef::Igniter_On()
{
    servo_igniter_on;
}
void Launch_Classdef::Igniter_Off()
{
    servo_igniter_off;
}

void Launch_Classdef::disable()
{
    Igniter_On();

    PID_Deliver_Angle[L].clean_intergral();
    PID_Deliver_Angle[R].clean_intergral();
    PID_Igniter_Angle.clean_intergral();

    PID_Deliver_Speed[L].clean_intergral();
    PID_Deliver_Speed[R].clean_intergral();
    PID_Igniter_Speed.clean_intergral();

    DeliverMotor[L].setMotorCurrentOut(0);
    DeliverMotor[R].setMotorCurrentOut(0);
    IgniterMotor.setMotorCurrentOut(0);
}

/* function prototypes -------------------------------------------------------*/

/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/