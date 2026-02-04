
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
   
}

void Missle_YawController_Classdef::update(float _yaw_target)
{
   
}

void Missle_YawController_Classdef::adjust()
{
}

void Missle_YawController_Classdef::disable()
{
   
}

void Missle_YawController_Classdef::yaw_out_motor_speed(){
   
}

void Missle_YawController_Classdef::yaw_state_machine(yaw_control_state_e *yaw_state,float RC_X,float RC_Y){
    
}


bool Missle_YawController_Classdef::isMotorAngleReached(float threshold)
{
    return std::abs(PID_Yaw_Angle.Error) <= threshold;
}

bool Missle_YawController_Classdef::is_Yaw_pid_Vision_stable(float threshold)
{
    return std::abs(PID_Yaw_Vision.Error) < threshold;
}
                               