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
    mymotor.angle_unit_convert = 1; // 角度单位转换
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
        mymotor_pid_spd.Current = motor_ctrl.get_motor_speed();
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
    mymotor.setMotorCurrentOut(mymotor_pid_spd.Out);
}

// ================= 状态查询 =================

bool motor_ctrl_driver::is_motor_at_target() {
    float err=abs(dm_motor_recdata.angle - target_motor_angle);
    return (err < threshold_motor_at_target);
}

float motor_ctrl_driver::get_motor_angle(){
    float angle = (float)(dm_motor_recdata.angle/65535.0f*360.0f/52.0f); // 换算成角度
    return angle;
}

float motor_ctrl_driver::get_motor_speed(){
    float speed = (float)(dm_motor_recdata.d_angle/65535.0f*360.0f/52.0f*500.0f); // 减速后，angle转一圈增量为65535*52,数据更新周期是2ms，换算成角度每秒
    return speed;
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

bool motor_ctrl_driver::update(uint32_t _unuse_id, uint8_t data[8])
{
    //if ((ID) != (data[0] & 0x0F))
        //return false;
    
    dm_motor_recdata.state = (data[0]) >> 4;
    encoder = (uint16_t)(data[1] << 8) | data[2];
	dm_motor_recdata.velocity =(data[3] << 4) | (data[4] >> 4); // (-45.0,45.0)
    dm_motor_recdata.torque = ((data[4] & 0xF) << 8) | data[5];   // (-10.0,10.0)
    dm_motor_recdata.T_mos = (float)(data[6]);
    dm_motor_recdata.T_motor = (float)(data[7]);
#if 1
    if (encoder_is_init)
    {
        if (encoder - last_encoder > encoder_max / 2)
            round_cnt--;
        else if (encoder - last_encoder < -encoder_max / 2)
            round_cnt++;
    }
    else
    {
        encoder_offset = encoder;
        encoder_is_init = true;
    }
    last_encoder = encoder;
    dm_motor_recdata.angle  = round_cnt * encoder_max + encoder - encoder_offset;
#else
    if (encoder - last_encoder > encoder_max / 2)
        round_cnt--;
    else if (encoder - last_encoder < -encoder_max / 2)
        round_cnt++;
    last_encoder = encoder;
    dm_motor_recdata.angle  = round_cnt * encoder_max + encoder;
#endif
    dm_motor_recdata.d_angle = dm_motor_recdata.angle - last_angle;
    last_angle = dm_motor_recdata.angle;
    return true;
}

