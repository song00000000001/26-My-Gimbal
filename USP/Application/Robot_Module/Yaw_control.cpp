
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
    //角度环
    if(is_Yaw_Init() == 1)
    {
        PID_Yaw_Angle.Current = YawMotor.getMotorTotalAngle();
        PID_Yaw_Angle.Adjust();
        PID_Yaw_Speed.Target = PID_Yaw_Angle.Out;
    }
    //速度环，校准过程中直接速度环控制
    else
    {
        PID_Yaw_Angle.Target = PID_Yaw_Angle.Current = YawMotor.getMotorTotalAngle();
        PID_Yaw_Angle.Adjust();
    }
    PID_Yaw_Speed.Current = YawMotor.getMotorSpeed();
    PID_Yaw_Speed.Adjust();
    
}

void Missle_YawController_Classdef::disable()
{
    //失控保护
    PID_Yaw_Angle.clean_intergral();
    PID_Yaw_Speed.clean_intergral();
    YawMotor.setMotorCurrentOut(0);
}

void Missle_YawController_Classdef::yaw_out_motor_speed(){
    YawMotor.setMotorCurrentOut(PID_Yaw_Speed.Out);
}


void Missle_YawController_Classdef::yaw_state_machine(yaw_control_state_e yaw_state,float yaw_manual_target){
    
    static float yaw_target = 0;
    static float yaw_correct_angle=0;        //yaw轴修正角

    switch (yaw_state)
    {
    case MANUAL_AIM:
        yaw_target -= yaw_manual_target * 0.002f;
        yaw_target = std_lib::constrain(yaw_target, -10.2f, 10.2f);
        Yawer.update(yaw_target);
        break;
    case CORRECT_AIM:
        //根据目标选择修正角度
        //固定修正值模式
        Yawer.update(yaw_correct_angle); // 更改Yaw轴角度
        break;
    case VISION_AIM:
        //视觉模式
        //todo
        {/*todo
        测试时，暂时不管视觉
        storage_base_angle = default_yaw_target[HitTarget];

        if (vision_recv_pack.ros == 1)
        {
            yaw_target += 0.0003;
        }
        if (vision_recv_pack.ros == 2)
        {
            yaw_target -= 0.0003;
        }
        if (vision_recv_pack.ros == 0)
        {
            yaw_target += 0;
        }
        yaw_target = std_lib::constrain(yaw_target, -10.2f, 10.2f);
        Yawer.update(yaw_target);
        default_yaw_target[HitTarget] = yaw_target;
        //计算电机pid
        Yawer.adjust();
        */
       }
        break;
    case YAW_CALIBRATING:
        //校准模式
        //进行校准，校准完成后，自动改变校准标志
        Yawer.calibration();
        if(Yawer.is_Yaw_Init()){
            Robot.Status.yaw_control_state = MANUAL_AIM; //校准完成后，进入手动模式
        }
        break;
    default:
        Yawer.disable();
        break;
    }

}



