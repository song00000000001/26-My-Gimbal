#pragma once

/*由于状态机任务cpp文件中需要使用这里驱动的函数
，所以加入extern "C"让C++编译器按C规则编译。
至于为什么不直接封装进驱动里，是因为函数里要用到状态类型，
我认为底层驱动不应该依赖上层模块的类型定义，所以选择在这里暴露接口。
*/
#include "global_data.h"
#include "tim.h"		//奇怪，放在extern "c"里vscode就不会报假错了

#ifdef __cplusplus
extern "C" {
#endif

/*
pwm引脚映射表
tim2_ch3 PC8
tim2挂在apb1上，时钟频率为84MHz，预分频器设置为0，计数器周期设置为105-1，则计数频率为84MHz/105=800khz，即每个计数周期为12.5us。
*/
#define arm_tim1 &htim2
#define arm_channel_1 TIM_CHANNEL_3

//void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim);	//DMA回调函数
void ws2312_show(uint8_t r, uint8_t g, uint8_t b); // 灯臂全部填充指定颜色
 
#ifdef __cplusplus
}
#endif

