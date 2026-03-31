#pragma once

#include "SRML.h"
#include "robot_config.h"

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

class motor_ctrl_driver
{
private:

public:
   
    uint16_t encoder,last_encoder,encoder_offset;
    bool encoder_is_init;
    const int32_t encoder_max = 65536; /* 码盘值最大值 */
    int32_t round_cnt,last_angle;
    
   
};
