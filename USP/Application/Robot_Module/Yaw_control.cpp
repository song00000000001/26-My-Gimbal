
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
    //PID_Yaw_Speed.I_SeparThresh = 0;
    //PID_Yaw_Speed.DeadZone = 0;
    mode_YAW = MODE_SPEED;
}

void Missle_YawController_Classdef::calibration()
{
    switch (Yaw_Init_flag)
    {
    case 0:
        PID_Yaw_Speed.Target = -calibration_speed.yaw_calibration_speed;
        
        if(SW_YAW_L_OFF)
        {
            YawMotor.baseAngle -= YawMotor.getMotorTotalAngle();
            Yaw_Init_flag = 1;
        }
        break;
    
    case 1:
        PID_Yaw_Speed.Target = calibration_speed.yaw_calibration_speed;
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
    if(mode_YAW == MODE_ANGLE)
    {
        PID_Yaw_Angle.Current = YawMotor.getMotorTotalAngle();
        PID_Yaw_Angle.Adjust();
        PID_Yaw_Speed.Target = PID_Yaw_Angle.Out;
    }
    //速度环，校准过程中直接速度环控制
    else if(mode_YAW == MODE_SPEED)
    {
        //这一步的目的应该是防止切到角度环后如果没有显示给target，就会停下，免得切状态后没有及时给一个设定值就乱跑。
        PID_Yaw_Angle.Target = PID_Yaw_Angle.Current = YawMotor.getMotorTotalAngle();
        PID_Yaw_Angle.Adjust();
    }
    else
    {
        YawMotor.setMotorCurrentOut(0);
        PID_Yaw_Angle.clean_intergral();
        PID_Yaw_Speed.clean_intergral();
        return;
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


void Missle_YawController_Classdef::yaw_state_machine(yaw_control_state_e yaw_state,float LX,float LY){
    
    switch (yaw_state)
    {
    case MANUAL_AIM:
        // 手动微调逻辑
        Launcher.target_igniter_angle-=LY * 0.02f;
        Launcher.target_igniter_angle=std_lib::constrain(Launcher.target_igniter_angle, IGNITER_MIN_POS, IGNITER_MAX_POS);
        yaw_target -= LX * 0.02f;
        yaw_target = std_lib::constrain(yaw_target, -10.2f, 10.2f);
        update(yaw_target);
        break;
    case CORRECT_AIM:
    {
        //读取调参板设置的发射数据
        //根据发射计数选择数据槽数组
        uint8_t slot_index=DartDataSlot[Robot.Status.dart_count]+1;

        //根据目标类型选择数据组数据
        //yaw数据
        //yaw_target=DartsData[slot_index].YawCorrectionAngle[HitTarget];
        yaw_target=DartsData[1].YawCorrectionAngle[0];
        yaw_target=std_lib::constrain(yaw_target, -10.2f, 10.2f);
        /*
        todo
        song
        这里用了外面的类，不太好改，就先放这里吧
        */
        //发射力度数据
        Launcher.target_igniter_angle=DartsData[1].Ignitergoal[0];
        Launcher.target_igniter_angle=std_lib::constrain(Launcher.target_igniter_angle, IGNITER_MIN_POS, IGNITER_MAX_POS);
        update(yaw_target); // 更改Yaw轴角度
    }
        break;
    case VISION_AIM:
        //视觉模式
        {
        /*todo
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
        update(yaw_target);
        default_yaw_target[HitTarget] = yaw_target;
        //计算电机pid
        adjust();
        */
        }
        disable();
        break;
    case YAW_CALIBRATING:
        //校准模式
        //进行校准，校准完成后，自动改变校准标志
        calibration();
        mode_YAW = MODE_SPEED; //校准过程中采用速度模式
        break;
    default:
        disable();
        break;
    }

}


bool Missle_YawController_Classdef::yaw_stall_check(float limit_output, float threhold_rpm, uint32_t time_ms)
{
    uint32_t now = xTaskGetTickCount();
    bool is_stalled = false;
    if (abs(PID_Yaw_Speed.Out) > limit_output && abs(YawMotor.getMotorSpeed()) < threhold_rpm) {
        if (stall_timer_yaw == 0) {
            stall_timer_yaw = now;
        }
        else if ((now - stall_timer_yaw) > time_ms) {
            is_stalled = true;
        }
    }
    else {
        stall_timer_yaw = 0;
    }

    return is_stalled;
}

bool Missle_YawController_Classdef::isMotorAngleReached(float threshold)
{
    return std::abs(PID_Yaw_Angle.Error) <= threshold;
}