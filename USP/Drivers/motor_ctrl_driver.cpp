#include "motor_ctrl_driver.h"
#include "robot_config.h"
#include "global_data.h"

// 构造函数
motor_ctrl_driver::motor_ctrl_driver(uint8_t id):
    mymotor(id)
{
    // 电机参数初始化 (极性、减速比)
    mymotor.Polarity = 1;
    //float deliver_ratio = (2 * PI * 18.62f) / (360 * 51);
    //mymotor.angle_unit_convert = deliver_ratio;
    mymotor.angle_unit_convert = 1;
    mymotor.speed_unit_convert = 1;

    mymotor_mode = MODE_SPEED; 
}


// ================= 动作接口 =================
void motor_ctrl_driver::adjust()
{
    
    if(mymotor_mode==MODE_ANGLE){
        // 串级PID: 位置环 -> 速度环
        target_motor_angle=std_lib::constrain(target_motor_angle,mymotor_limit.lower_limit,mymotor_limit.upper_limit);
        mymotor_pid_pos.Target = target_motor_angle;
        mymotor_pid_pos.Current = mymotor.getMotorTotalAngle();
        mymotor_pid_pos.Adjust();
        //速度环的输入为角度环输出
        mymotor_pid_spd.Target = mymotor_pid_pos.Out;
        //速度环
        mymotor_pid_spd.Current = mymotor.getMotorSpeed();
        mymotor_pid_spd.Adjust();
    }
    else if(mymotor_mode==MODE_SPEED){
        mymotor_pid_spd.Current = mymotor.getMotorSpeed();
        mymotor_pid_spd.Adjust();
    }
    else{//意外情况,一般不会进入。这里的接口外部不应该调用，而是通过motor_output的enable参数控制。
        mymotor_pid_pos.Target=mymotor_pid_pos.Current;
        mymotor_pid_pos.Out=0;
        mymotor_pid_spd.Target=0;
        mymotor_pid_spd.Out=0;
        mymotor_pid_pos.clean_intergral();
        mymotor_pid_spd.clean_intergral();
        mymotor.setMotorCurrentOut(0);
    }
}

// 输出所有电机控制电流
void motor_ctrl_driver::motor_output(bool enable){
    if(enable==false){
        mymotor_pid_pos.Target=mymotor_pid_pos.Current;
        mymotor_pid_pos.Out=0;
        mymotor_pid_spd.Target=0;
        mymotor_pid_spd.Out=0;
        mymotor_pid_pos.clean_intergral();
        mymotor_pid_spd.clean_intergral();
        mymotor.setMotorCurrentOut(0);
        return;
    }
    adjust();
    mymotor.setMotorCurrentOut(mymotor_pid_spd.Out);
}

// ================= 状态查询 =================

bool motor_ctrl_driver::is_motor_at_target() {
    float err=abs(mymotor.getMotorTotalAngle() - target_motor_angle);
    return (err < threshold_motor_at_target);
}

float motor_ctrl_driver::get_motor_angle(){
    return mymotor.getMotorTotalAngle();
}

float motor_ctrl_driver::get_motor_speed(){
    return mymotor.getMotorSpeed();
}

// 设置电机目标角度
void motor_ctrl_driver::set_motor_target_angle(float angle){
    target_motor_angle=angle;
}
// 设置电机角度限幅
void motor_ctrl_driver::set_motor_angle_limit(float lower_limit,float upper_limit){
    mymotor_limit.lower_limit = lower_limit;
    mymotor_limit.upper_limit = upper_limit;
}
// 设置电机模式
void motor_ctrl_driver::set_motor_mode(Control_Mode_e mode){
    mymotor_mode = mode;
}
// 设置电机目标速度
void motor_ctrl_driver::set_motor_target_speed(float speed){
    if(mymotor_mode!=MODE_SPEED){
        return;
    }
    mymotor_pid_spd.Target = speed;
}