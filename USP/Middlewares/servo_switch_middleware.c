#include "robot_config.h"

//由于在cpp调用c总会出现警告，所以把任务中会调用c的语句都集中在该c文件中
//为了方便修改参数，底层调用的函数都写成宏定义，放在robot_config.h中
//这里只写动作组合任务

/**
 * @brief 装填状态控制任务，夹爪控制部分
 * @parma None
 * @return None
 */

//三个夹爪全部夹紧
void Loader_Clamps_ClampAll(void)
{
    servo_loader_clamp1;    // 一号夹爪夹紧
    servo_loader_clamp2;    // 二号夹爪夹紧
    servo_loader_clamp3;    // 三号夹爪夹紧
}
//三个夹爪全部松开
void Loader_Clamps_ReleaseAll(void)
{
    servo_loader_release1;   // 一号夹爪松开
    servo_loader_release2;   // 二号夹爪松开
    servo_loader_release3;   // 三号夹爪松开 
}
//松开1号夹爪，夹紧2号和3号夹爪
void Loader_Clamps_Release1(void)
{
    servo_loader_release1;   // 一号夹爪松开
    servo_loader_clamp2;    // 二号夹爪夹紧
    servo_loader_clamp3;    // 三号夹爪夹紧
}
//松开2号夹爪，夹紧1号和3号夹爪
void Loader_Clamps_Release2(void)
{
    servo_loader_clamp1;    // 一号夹爪夹紧
    servo_loader_release2;   // 二号夹爪松开
    servo_loader_clamp3;    // 三号夹爪夹紧
}
//松开3号夹爪，夹紧1号和2号夹爪
void Loader_Clamps_Release3(void)
{
    servo_loader_clamp1;    // 一号夹爪夹紧
    servo_loader_clamp2;    // 二号夹爪夹紧
    servo_loader_release3;   // 三号夹爪松开 
}

//临时测试舵机动作
void test_servo_action()
{
    servo_igniter_on;
    HAL_Delay(1000);
    servo_igniter_off;
    HAL_Delay(1000);

}
