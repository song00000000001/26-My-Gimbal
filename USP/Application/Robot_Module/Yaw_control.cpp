
#include "Yaw_control.h"
#include "global_data.h"
#include "robot_config.h"

//视觉debug参数,设置yaw轴扫描幅度和速度,还有视觉微调速度,用结构体封装
typedef struct {
    float yaw_scan_range[2]; //扫描幅度,单位度,数组[最小值,最大值],默认{-5.0f,5.0f}
    float yaw_scan_speed; //扫描速度,单位
    float yaw_vision_fine_tune_speed; //视觉微调速度,单位
} VisionDebugParams_t;

VisionDebugParams_t vision_debug_params = {
    .yaw_scan_range = {-3.0f, 3.0f}, //扫描幅度
    .yaw_scan_speed = 0.003f, //扫描速度
    .yaw_vision_fine_tune_speed = 0.0003f //视觉微调速度
};

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
    _yaw_target = std_lib::constrain(_yaw_target, YAW_MIN_ANGLE, YAW_MAX_ANGLE);
    /*todo
    song
    这里的公式需要重新推导一下  
    */

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
    #if 0 //由于限位开关延迟问题，容易误触发，这里先注释掉
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
    #endif
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
    //由于没校准就触发以下状态会造成bug，先加个校准完成的判断
    if(Robot.Flag.Status.rc_connected){
        if(!is_Yaw_Init())
        {
            *yaw_state = YAW_CALIBRATING;
        }
        else{
            if(Robot.Flag.Status.vision_connected)//视觉连接
            {
                *yaw_state = YAW_VISION_AIM;
            }
            else if(Robot.Flag.Status.tool_panel_connected)//调参板连接
            {
                *yaw_state = YAW_CORRECT_AIM;
            }
            else
                *yaw_state=YAW_MANUAL_AIM;
        }
    }
    else
        *yaw_state=YAW_DISABLE_MOTOR;
    
    switch (*yaw_state)
    {
    case YAW_MANUAL_AIM:
        // 手动微调逻辑
        //由于摇杆复用,在紧急预案时需要ban掉手动控制igniter
        if(!Robot.Flag.Status.emergency_override){
            Launcher.target_igniter_angle-=RC_Y * IGNITER_MANUAL_SPEED;
            yaw_target -= RC_X * YAW_MANUAL_SPEED;
            update(yaw_target);
        }
        break;

    case YAW_CORRECT_AIM:
        {
            //读取调参板设置的发射数据
            //根据发射计数选择数据槽数组
            uint8_t slot_index=1;//测试用，固定数据槽1
            
            /*todo
            song
            这里因为目前没有四发镖,做不到四发四参测试,所以先固定用数据槽1的数据进行测试
            以后有条件了再改回来
            */
            if(Debugger.four_dart_four_params_enable)
            {
                slot_index=(Robot.Status.dart_count-1)%4+1;
                //第一发就是[(1-1)%4=0],读取第[0]号数据槽,也就是数据内容1
            }

            //根据目标类型选择数据组数据
            yaw_target=DartsData[slot_index].YawCorrectionAngle[0];//这里的0是指前哨站，1是基地。由于调参板默认先显示前哨站数据，所以先用0。实际上只打基地。这个约定要沟通好。
            /*
            todo
            song
            这里用了外面的类，不太好改，就先放这里吧
            */
            //发射力度数据
            Launcher.target_igniter_angle=DartsData[slot_index].Ignitergoal[0];
            update(yaw_target); // 更改Yaw轴角度

            //记录角度和力度变化，并且加上当前发射计数,将yaw和行程合并输出
            static float last_yaw_target=0;
            static float last_igniter_target=0;
            if(std::abs(last_yaw_target - yaw_target) > 0.01f || std::abs(last_igniter_target - Launcher.target_igniter_angle) > 0.01f)
            {
                LOG_INFO("Dart num:%d Target Update,date slot: %d", Robot.Status.dart_count, slot_index);
                LOG_INFO("Yaw Target: %.2f -> %.2f,Igniter Target: %.2f -> %.2f", 
                    last_yaw_target, yaw_target, last_igniter_target, Launcher.target_igniter_angle);
                last_yaw_target = yaw_target;
                last_igniter_target = Launcher.target_igniter_angle;
            }
        }
        break;

    case YAW_VISION_AIM:
        //视觉模式
        {
            //非紧急预案
            if(!Robot.Flag.Status.emergency_override){
                //手动启用视觉
                if(Debugger.debug_vision_enable==2){
                    //记录视觉识别状态变化
                    static uint8_t last_vision_target_mode = 0;
                    if (last_vision_target_mode != vision_recv_pack.target_mode) {
                        if(vision_recv_pack.target_mode!=0){
                            LOG_INFO("Vision Target Acquired: Yaw %.2f", vision_recv_pack.target_yaw);
                            Debugger.buzzer_beep_count=3; //视觉识别到目标鸣叫
                        }
                        else{
                            LOG_INFO("Vision Target Lost");
                        }
                        last_vision_target_mode = vision_recv_pack.target_mode;
                    }
                    if(vision_recv_pack.target_mode==0)//若视觉未识别到引导灯，则先自行扫描
                    {
                        static int8_t direction_temp=1;
                        if(yaw_target>=vision_debug_params.yaw_scan_range[1])
                        {
                            direction_temp=-1;
                        }
                        else if(yaw_target<=vision_debug_params.yaw_scan_range[0])
                        {
                            direction_temp=1;
                        }
                        yaw_target+=vision_debug_params.yaw_scan_speed*direction_temp;
                    }
                    else//识别到目标则微调
                    {
                        if (vision_recv_pack.ros == 1)
                        {
                            yaw_target += vision_debug_params.yaw_vision_fine_tune_speed;
                        }
                        if (vision_recv_pack.ros == 2)
                        {
                            yaw_target -= vision_debug_params.yaw_vision_fine_tune_speed;
                        }
                        if (vision_recv_pack.ros == 0)
                        {
                            yaw_target += 0;
                        }
                        //记录视觉微调状态变化
                        static uint8_t last_vision_ros=0;
                        if(last_vision_ros!=vision_recv_pack.ros)
                        {
                            if(vision_recv_pack.ros==4)    //目标稳定
                            {
                                LOG_INFO("Vision Yaw Target Stable: %.2f", yaw_target);
                                Debugger.buzzer_beep_count=4; //目标稳定鸣叫
                            }
                            last_vision_ros = vision_recv_pack.ros;
                        }
                    }
                }
                else if(Robot.Flag.Status.tool_panel_connected){
                    //视觉关闭,调参板连接，优先用调参板数据
                    uint8_t slot_index=1;//测试用，固定数据槽1
                    if(Debugger.four_dart_four_params_enable)slot_index=(Robot.Status.dart_count-1)%4+1;//第一发就是[(1-1)%4=0],读取第[0]号数据槽,也就是数据内容1
                    yaw_target=DartsData[slot_index].YawCorrectionAngle[0];//这里的0是指前哨站，1是基地。由于调参板默认先显示前哨站数据，所以先用0。实际上只打基地。这个约定要沟通好。
                }
                else{
                    //视觉关闭,调参板未连接，使用遥控器微调
                    yaw_target -= RC_X * YAW_MANUAL_SPEED;
                }
            }
            update(yaw_target);
            adjust();

            if(!Robot.Flag.Status.emergency_override){
                if(Robot.Flag.Status.tool_panel_connected){
                    //视觉模式下，调参板可以调整igniter目标位置
                    uint8_t slot_index=1;//测试用，固定数据槽1
                    if(Debugger.four_dart_four_params_enable)slot_index=(Robot.Status.dart_count-1)%4+1;//第一发就是[(1-1)%4=0],读取第[0]号数据槽,也就是数据内容1
                    Launcher.target_igniter_angle=DartsData[slot_index].Ignitergoal[0];
                }
                else{
                    Launcher.target_igniter_angle-=RC_Y * IGNITER_MANUAL_SPEED;
                }
            }
        }
        break;
    case YAW_CALIBRATING:
        //校准模式
        //进行校准，校准完成后，自动改变校准标志
        calibration();
        mode_YAW = MODE_SPEED; //校准过程中采用速度模式
        //日志记录校准状态
        static uint8_t last_yaw_calib_flag = 0;
        if (last_yaw_calib_flag != Yaw_Init_flag) {
            LOG_INFO("Yaw Calibration State Change: %d -> %d", last_yaw_calib_flag, Yaw_Init_flag);
            switch (Yaw_Init_flag)
            {
            case 0:
                LOG_INFO("Yaw Calibration: Moving to Left Limit");
                break;
            case 1:
                LOG_INFO("Yaw Calibration: Left Limit Reached, Moving to Right Limit");
                break;
            case 2:
                LOG_INFO("Yaw Calibration: Right Limit Reached, Calibration Complete");
                break;
            default:
                break;
            }
            last_yaw_calib_flag = Yaw_Init_flag;
        }
        break;
    case YAW_DISABLE_MOTOR:
    default:
        disable();
        break;
    }

    //日志记录yaw子状态
    static yaw_control_state_e last_yaw_state = YAW_MANUAL_AIM;
    if (last_yaw_state != *yaw_state) {
        #if enum_X_Macros_disable
        LOG_INFO("Yaw Control State Change: %d -> %d", last_yaw_state, *yaw_state);
        #else
        LOG_INFO("Yaw Control State Change: %s -> %s", Yaw_Control_State_To_Str(last_yaw_state), Yaw_Control_State_To_Str(*yaw_state));
        #endif

        last_yaw_state = *yaw_state;
    }

}


bool Missle_YawController_Classdef::isMotorAngleReached(float threshold)
{
    return std::abs(PID_Yaw_Angle.Error) <= threshold;
}