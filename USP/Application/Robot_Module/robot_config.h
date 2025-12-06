#pragma once
#include "SRML.h"
#include "stm32f4xx_hal.h"
#include "tim.h"    
#include "stm32f4xx_hal_tim.h"

/* ================= 电机 ID 定义 ================= */
#define ID_DELIVER_L                  4     // 左同步轮
#define ID_DELIVER_R                  3     // 右同步轮
#define ID_IGNITER                    1     // 扳机丝杆   
#define ID_YAW                        2     // 云台偏航
#define ID_LOADER                     5     // 装填转盘，测试id为1

/* ================= 电机极性定义 ================= */
// 1 或 -1
#define POLARITY_DELIVER_L            -1    // 左同步轮
#define POLARITY_DELIVER_R            1     // 右同步轮
#define POLARITY_IGNITER              1     // 扳机丝杆
#define POLARITY_YAW                  1     // 云台偏航
#define POLARITY_LOADER               1     // 装填转盘

/* ================= 机械常数 & 目标值 ================= */
// Deliver (滑块)
#define POS_DELIVER_INIT        -10.0f      // 复位位置
#define POS_DELIVER_PULL        -640.0f     // 拉栓位置
#define SPEED_DELIVER_INIT      1000.0f     // 校准速度

// Igniter (扳机丝杆)
#define POS_IGNITER_INIT        3.0f        // 复位位置
#define SPEED_IGNITER_INIT      2000.0f     // 校准速度

// Loader (装填转盘) - 编码器值
#define POS_LOADER_READY        0.0f
#define POS_LOADER_DART1        2750.0f
#define POS_LOADER_DART2        1340.0f
#define POS_LOADER_DART3        4090.0f
#define POS_LOADER_DART4        6820.0f

#define INIT_SPEED_DELIVER            6000
#define INIT_SPEED_IGNITER            -4500
#define INIT_SPEED_YAW                -500

//以下用到了c语言函数,需要加extern "C"修饰
#ifdef __cplusplus
extern "C"{
#endif

/* ==舵机宏== */
//扳机舵机可能是tim1,ch1,待检查
#define servo_igniter_on        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 205) // 扳机舵机解锁
#define servo_igniter_off       __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 320) // 扳机舵机锁止

#define servo_loader_clamp1     __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, 53)   // 一号夹爪夹紧
#define servo_loader_release1   __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, 100)  // 一号夹爪松开
#define servo_loader_clamp2     __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, 126)  // 二号夹爪夹紧
#define servo_loader_release2   __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, 170)  // 二号夹爪松开
#define servo_loader_clamp3     __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, 288)  // 三号夹爪夹紧
#define servo_loader_release3   __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, 360)  // 三号夹爪松开 

//舵机动作组
void Loader_Clamps_ClampAll(void);
void Loader_Clamps_ReleaseAll(void);
void Loader_Clamps_Release1(void);
void Loader_Clamps_Release2(void);
void Loader_Clamps_Release3(void);
void test_servo_action();

// ================= 开关引脚定义 =================
#define SW_DELIVER_L_Pin              GPIO_PIN_4
#define SW_DELIVER_L_GPIO_Port        GPIOC
#define SW_DELIVER_R_Pin              GPIO_PIN_5
#define SW_DELIVER_R_GPIO_Port        GPIOA
#define SW_IGNITER_Pin                GPIO_PIN_14
#define SW_IGNITER_GPIO_Port          GPIOC

#define SW_YAW_R_Pin                  GPIO_PIN_6
#define SW_YAW_R_GPIO_Port            GPIOA
#define SW_YAW_L_Pin                  GPIO_PIN_7
#define SW_YAW_L_GPIO_Port            GPIOA

#define READ_SW_DELIVER_L  HAL_GPIO_ReadPin(SW_DELIVER_L_GPIO_Port,SW_DELIVER_L_Pin)
#define READ_SW_DELIVER_R HAL_GPIO_ReadPin(SW_DELIVER_R_GPIO_Port,SW_DELIVER_R_Pin)
#define READ_SW_IGNITER HAL_GPIO_ReadPin(SW_IGNITER_GPIO_Port,SW_IGNITER_Pin)

#define SW_DELIVER_L_OFF (HAL_GPIO_ReadPin(SW_DELIVER_L_GPIO_Port, SW_DELIVER_L_Pin))==GPIO_PIN_RESET
#define SW_DELIVER_R_OFF (HAL_GPIO_ReadPin(SW_DELIVER_R_GPIO_Port, SW_DELIVER_R_Pin))==GPIO_PIN_RESET
#define SW_IGNITER_OFF (HAL_GPIO_ReadPin(SW_IGNITER_GPIO_Port, SW_IGNITER_Pin))==GPIO_PIN_RESET

#define SW_YAW_R_OFF (HAL_GPIO_ReadPin(SW_YAW_R_GPIO_Port, SW_YAW_R_Pin))==GPIO_PIN_RESET
#define SW_YAW_L_OFF (HAL_GPIO_ReadPin(SW_YAW_L_GPIO_Port, SW_YAW_L_Pin))==GPIO_PIN_RESET


#ifdef __cplusplus
}
#endif

/* PIDinit */
#define PID_DELIVER_DIFF_PARAM 0.5f, 0.0f, 0.0f, 8000, 16000
#define PID_DELIVER_SPD_PARAM  20.0f, 2.0f, 0.0f, 8000, 16380
#define PID_DELIVER_ANG_PARAM  800.f, 0.0, 0.0, 1000, 8000
#define PID_IGNITER_SPD_PARAM  15.0, 0.0, 0.0, 3000, 16000
#define PID_IGNITER_ANG_PARAM  3000.0, 0.0, 0.0, 3000, 7000

// Yaw轴
#define PID_YAW_SPD_PARAM       20.0f, 0.0f, 0.0f, 0.0f, 18000.0f
#define PID_YAW_POS_PARAM       15.0f, 0.0f, 0.0f, 0.0f, 300.0f

// 装填转盘
#define PID_LOADER_SPD_PARAM    8.0f, 1.0f, 0.0f, 100.0f, 16000.0f
#define PID_LOADER_POS_PARAM    4.0f, 0.0f, 0.1f, 100.0f, 1200.0f



