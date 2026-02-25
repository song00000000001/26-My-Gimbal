#pragma once

#include "SRML.h"

typedef struct motor_angle_limit_t{
    float lower_limit;
    float upper_limit;
} motor_angle_limit_t;

// 定义控制模式
enum Control_Mode_e {
    MODE_SPEED=0,       // 速度环
    MODE_ANGLE,      // 角度环
    MODE_ERROR      // 失能
} ;

struct dm_motor_recdata_t
{
    int16_t angle = 0;   // 角度编码值
    uint8_t state = 0; // 电机状态  
    int16_t velocity=0;    // 当前速度
    int16_t torque = 0;  // 当前力矩
    uint8_t T_mos = 0;   // 表示驱动上 MOS 的平均温度
    uint8_t T_motor = 0; // 表示电机内部线圈的平均温度
};

class motor_ctrl_driver
{
private:

public:
    dm_motor_recdata_t dm_motor_recdata;
    Motor_DM_classdef mymotor;
    uint16_t encoder,last_encoder,encoder_offset;
    bool encoder_is_init;
    const int32_t encoder_max = 65536; /* 码盘值最大值 */
    int32_t round_cnt;

    #if 0
    // PID 对象
    myPID mymotor_pid_spd,mymotor_pid_pos;    

    //抽象电机对象
    //abstractMotor<Motor_C620> mymotor;

    Control_Mode_e mymotor_mode;    
    float target_motor_angle;    
    float threshold_motor_at_target=1.0f; // 角度环目标到达阈值
    motor_angle_limit_t mymotor_limit;

    // 根据电机模式（角度环，速度环，失能）调用 PID 计算
    


    void adjust();

    // 输出所有电机控制电流
    void motor_output(bool enable);

    // 是否到达目标位置
    bool is_motor_at_target();
    // 获取电机当前信息
    float get_motor_angle();
    float get_motor_speed();
    // 设置电机目标角度
    void set_motor_target_angle(float angle);
    // 设置电机角度限幅
    void set_motor_angle_limit(float lower_limit,float upper_limit);
    // 设置电机模式
    void set_motor_mode(Control_Mode_e mode);

    #endif
    motor_ctrl_driver(uint8_t id);
    // 设置电机目标速度
    void motor_pack_dm10010(CAN_COB &txPack,float speed);
    // 使能/失能电机
    void enable_motor(bool enable);
    // 更新电机状态，解析 CAN 数据
    bool update(uint32_t _unuse_id, uint8_t data[8]);
};
