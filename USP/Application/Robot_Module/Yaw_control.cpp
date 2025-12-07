
#include "Yaw_control.h"
#include "global_data.h"
#include "robot_config.h"


Missle_YawController_Classdef::Missle_YawController_Classdef(uint8_t _ID_YAW)
: YawMotor(_ID_YAW)
{
    YawMotor.angle_unit_convert = 4 / 360.f;
    PID_Yaw_Angle.SetPIDParam(15, 0, 0, 0, 300);
    PID_Yaw_Angle.I_SeparThresh = 8;
    PID_Yaw_Angle.DeadZone = 0.01f;
    PID_Yaw_Speed.SetPIDParam(20, 0, 0, 0, 18000);
}

void Missle_YawController_Classdef::calibration()
{
    switch (Yaw_Init_flag)
    {
    case 0:
        PID_Yaw_Speed.Target = -INIT_SPEED_YAW;
        
        if(SW_YAW_L_OFF)
        {
            YawMotor.baseAngle -= YawMotor.getMotorTotalAngle();
            Yaw_Init_flag = 1;
        }
        break;
    
    case 1:
        PID_Yaw_Speed.Target = INIT_SPEED_YAW;
        if(SW_YAW_R_OFF)
        {
            MAX_YAW_ANGLE = YawMotor.getMotorTotalAngle();
            Yaw_Init_flag = 2;
            PID_Yaw_Angle.Target = 0.5 * MAX_YAW_ANGLE;
        }
        break;
    
    default:
        PID_Yaw_Speed.Target = 0;
        break;
    }
}

void Missle_YawController_Classdef::update(float _yaw_target)
{
    PID_Yaw_Angle.Target = 0.5f * MAX_YAW_ANGLE + 532.5f * tanf(_yaw_target / 180.f * PI);//0.5MAX_YAW_ANGLE后的部分需要重新计算
    PID_Yaw_Angle.Target = std_lib::constrain(PID_Yaw_Angle.Target, MAX_YAW_ANGLE, 0.0f);
}

void Missle_YawController_Classdef::adjust()
{
    if(is_Yaw_Init() == 1)
    {
        PID_Yaw_Angle.Current = YawMotor.getMotorTotalAngle();
        PID_Yaw_Angle.Adjust();
        PID_Yaw_Speed.Target = PID_Yaw_Angle.Out;
    }
    else
    {
        PID_Yaw_Angle.Target = PID_Yaw_Angle.Current = YawMotor.getMotorTotalAngle();
        PID_Yaw_Angle.Adjust();
    }
    PID_Yaw_Speed.Current = YawMotor.getMotorSpeed();
    PID_Yaw_Speed.Adjust();
    YawMotor.setMotorCurrentOut(PID_Yaw_Speed.Out);
}

void Missle_YawController_Classdef::disable()
{
    //失控保护
    PID_Yaw_Angle.clean_intergral();
    PID_Yaw_Speed.clean_intergral();
    YawMotor.setMotorCurrentOut(0);
}
