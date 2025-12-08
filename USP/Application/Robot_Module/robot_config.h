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

/* 常量定义 */
// 归零时的速度,速度环输入
#define INIT_SPEED_YAW        -600
#define DELIVER_HOME_SPEED   6000
#define IGNITER_HOME_SPEED   -2000

//角度环输出限幅
#define DELIVER_MAX_SPEED    8000
#define IGNITER_MAX_SPEED    6000

#define DELIVER_OFFSET_POS   -20  // 碰到开关后设置的初始坐标
#define IGNITER_OFFSET_POS   3   // 复位位置

#define POS_BUFFER -20       // 复位位置
#define POS_IGNITER 100     //行程电机位置
#define POS_BOTTOM -620      // 拉栓位置

//以下用到了c语言函数,需要加extern "C"修饰
#ifdef __cplusplus
extern "C"{
#endif

/* ==舵机宏== */

#define servo_igniter_unlock    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 170) // 扳机舵机解锁      ,120卡住,180ok
#define servo_igniter_lock      __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 270) // 扳机舵机锁止

/*ttodo
以下只有1号舵机响应,其他的都不对,而且测试时用不上,所以先不管
后续检查
*/
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
#define READ_SW_YAW_R HAL_GPIO_ReadPin(SW_YAW_R_GPIO_Port,SW_YAW_R_Pin)
#define READ_SW_YAW_L HAL_GPIO_ReadPin(SW_YAW_L_GPIO_Port,SW_YAW_L_Pin)

#define SW_DELIVER_L_OFF (HAL_GPIO_ReadPin(SW_DELIVER_L_GPIO_Port, SW_DELIVER_L_Pin))==GPIO_PIN_RESET
#define SW_DELIVER_R_OFF (HAL_GPIO_ReadPin(SW_DELIVER_R_GPIO_Port, SW_DELIVER_R_Pin))==GPIO_PIN_RESET
#define SW_IGNITER_OFF (HAL_GPIO_ReadPin(SW_IGNITER_GPIO_Port, SW_IGNITER_Pin))==GPIO_PIN_SET

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

/* 自检掩码定义 */
#define MASK_DELIVER_L  (1 << 0)
#define MASK_DELIVER_R  (1 << 1)
#define MASK_IGNITER    (1 << 2)
#define MASK_YAW_L      (1 << 3) 
#define MASK_YAW_R      (1 << 4)

// 定义全部通过的目标值 (0b11111 = 0x1F)
#define MASK_ALL_PASSED (MASK_DELIVER_L | MASK_DELIVER_R | MASK_IGNITER | MASK_YAW_L | MASK_YAW_R)

void yaw_state_machine();