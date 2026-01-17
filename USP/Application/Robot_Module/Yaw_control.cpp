
#include "Yaw_control.h"
#include "global_data.h"
#include "robot_config.h"


Missle_YawController_Classdef::Missle_YawController_Classdef(uint8_t _ID_YAW)
: YawMotor(_ID_YAW)
{
    YawMotor.angle_unit_convert = 4 / 360.f;
    
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
            //这里的MAX也有点奇怪,后面用constrain的时候看起来是min
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
    PID_Yaw_Angle.Target = std_lib::constrain(PID_Yaw_Angle.Target, MAX_YAW_ANGLE, 0.0f);//这里的限幅有点奇怪，不过能用就不管它。
}

void Missle_YawController_Classdef::adjust()
{
    //角度环
    if(mode_YAW == MODE_ANGLE)
    {
        PID_Yaw_Angle.Current = YawMotor.getMotorTotalAngle();
        PID_Yaw_Angle.Adjust();
        // 速度环目标由角度环输出
        PID_Yaw_Speed.Target = PID_Yaw_Angle.Out;
    }
    //速度环，校准过程中直接速度环控制
    else if(mode_YAW == MODE_SPEED)
    {
        //这一步的目的应该是切到角度环后默认会停留，免得切状态后没有给一个设定值却直接跑到0点（参数默认为0）。
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

    //意外触发限位开关log
    if(Yaw_Init_flag == 2){
        if(SW_YAW_L_OFF)
        {
            LOG_WARN("Yaw Left Limit Switch Triggered when calibrated");
        }
        if(SW_YAW_R_OFF)
        {
            LOG_WARN("Yaw Right Limit Switch Triggered when calibrated");
        }
    }
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

void Missle_YawController_Classdef::yaw_state_machine(yaw_control_state_e *yaw_state,float RC_X,float RC_Y){
    /*todo
    song
    修改yaw轴子状态机状态控制，由摇杆控制转为设备连接状态控制。即如果调参板连上了，就用调参板,否则用遥控器。
    如果视觉连上了，就优先用视觉，否则检查调参板，最后才是遥控器，如果遥控也没连上，就失能。
    */
    if(Robot.Flag.Status.vision_connected)//视觉连接
    {
        *yaw_state = VISION_AIM;
    }
    else if(Robot.Flag.Status.tool_panel_connected)//调参板连接
    {
        *yaw_state = CORRECT_AIM;
    }
    /*
    else if(DR16_Snap.S2==)//拨动摇杆
    {
        *yaw_state = MANUAL_AIM;
    }
    else
    {
        *yaw_state = DISABLE_MOTOR;
    }   
    */
    switch (*yaw_state)
    {
    case MANUAL_AIM:
        // 手动微调逻辑
        Launcher.target_igniter_angle-=RC_Y * 0.02f;
        //这里直接用角度（实际上是距离）限幅，因为丝杆和滑块电机可以得到简单的线性映射关系，抽象电机库可以直接配置映射参数。
        Launcher.target_igniter_angle=std_lib::constrain(Launcher.target_igniter_angle, IGNITER_MIN_POS, IGNITER_MAX_POS);
        yaw_target -= RC_X * 0.02f;
        //这里的限幅和其他电机不同,用的是镖架整体朝向的角度值,在update里进行三角函数转换后还会对电机编码器角度再限幅一次。
        yaw_target = std_lib::constrain(yaw_target, -10.2f, 10.2f);
        update(yaw_target);
        break;
    case CORRECT_AIM:
    {
        //读取调参板设置的发射数据
        //根据发射计数选择数据槽数组
        uint8_t slot_index=DartDataSlot[Robot.Status.dart_count%4]+1;
        /*
        todo
        song
        这里没用参数映射表,而是统一用1号位参数,方便调试。
        后期有装填后再改4发一发一参。
        */

        //根据目标类型选择数据组数据
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
            song
            测试视觉
        */
        if(vision_recv_pack.target_mode==0)//若视觉未识别到引导灯，则先自行扫描
        {
            int8_t direction_temp=1;
            if(yaw_target>=3)
            {
                direction_temp=-1;
            }
            else if(yaw_target<=-3)
            {
                direction_temp=1;
            }
            yaw_target+=0.012f*direction_temp;
        }
        else//识别到目标则微调
        {
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
        }
        //统一限幅并更新pid
        yaw_target = std_lib::constrain(yaw_target, -10.2f, 10.2f);
        update(yaw_target);
        //计算电机pid
        adjust();
        }
        break;
    case YAW_CALIBRATING:
        //校准模式
        //进行校准，校准完成后，自动改变校准标志
        calibration();
        mode_YAW = MODE_SPEED; //校准过程中采用速度模式
        break;
    case DISABLE_MOTOR:
    default:
        disable();
        break;
    }

    //日志记录yaw子状态
    static yaw_control_state_e last_yaw_state = MANUAL_AIM;
    if (last_yaw_state != *yaw_state) {
        LOG_INFO("Yaw Control State Change: %d -> %d", last_yaw_state, *yaw_state);
        last_yaw_state = *yaw_state;
    }

}


bool Missle_YawController_Classdef::isMotorAngleReached(float threshold)
{
    return std::abs(PID_Yaw_Angle.Error) <= threshold;
}