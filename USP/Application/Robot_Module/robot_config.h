#pragma once
#include "SRML.h"
#include "tim.h"    

#ifdef __cplusplus
extern "C" {
#endif

//保守测试参数切换宏
#define CONSERVATIVE_TEST_PARAMS 1

/* ================= 电机 ID 定义 ================= */
#define ID_YAW                        2     // 云台偏航,can2
#define ID_DELIVER_L                  1     // 左同步轮,can1
#define ID_DELIVER_R                  3     // 右同步轮,can1
#define ID_IGNITER                    4     // 扳机丝杆,can1 

/* ================= 电机极性定义 ================= */
// 1 或 -1
#define POLARITY_DELIVER_L            -1    // 左同步轮
#define POLARITY_DELIVER_R            1     // 右同步轮
#define POLARITY_IGNITER              1     // 扳机丝杆
#define POLARITY_YAW                  1     // 云台偏航
#define POLARITY_LOADER               1     // 装填转盘

/* ================= 机械常数 & 目标值 ================= */

/* 常量定义 */

#define DELIVER_OFFSET_POS   -5  // deliver碰到开关后设置的初始坐标
#define IGNITER_OFFSET_POS   3   // igniter复位位置

#define POS_BUFFER -20          //缓冲区位置
#define POS_BOTTOM -650         //拉栓位置

#define POS_IGNITER 90         //默认力度,igniter位置

//igniter最小/大位置
#define IGNITER_MIN_POS 2.0f
#define IGNITER_MAX_POS 200.0f
//deliver最小/大位置
#define POS_DELIVER_MIN -655.0f
#define POS_DELIVER_MAX -5.0f

//以下用到了c语言函数,需要加extern "C"修饰
#ifdef __cplusplus
extern "C"{
#endif

//舵机动作组

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
/*轻触开关映射表
deliver_L:PC4
deliver_R:PA5
igniter:PC14
yaw_R:PA6
yaw_L:PA7
*/
#define READ_SW_DELIVER_L  HAL_GPIO_ReadPin(SW_DELIVER_L_GPIO_Port,SW_DELIVER_L_Pin)
#define READ_SW_DELIVER_R HAL_GPIO_ReadPin(SW_DELIVER_R_GPIO_Port,SW_DELIVER_R_Pin)
#define READ_SW_IGNITER HAL_GPIO_ReadPin(SW_IGNITER_GPIO_Port,SW_IGNITER_Pin)
#define READ_SW_YAW_R HAL_GPIO_ReadPin(SW_YAW_R_GPIO_Port,SW_YAW_R_Pin)
#define READ_SW_YAW_L HAL_GPIO_ReadPin(SW_YAW_L_GPIO_Port,SW_YAW_L_Pin)

#define SW_DELIVER_L_OFF (HAL_GPIO_ReadPin(SW_DELIVER_L_GPIO_Port, SW_DELIVER_L_Pin))==GPIO_PIN_RESET
#define SW_DELIVER_R_OFF (HAL_GPIO_ReadPin(SW_DELIVER_R_GPIO_Port, SW_DELIVER_R_Pin))==GPIO_PIN_RESET
#define SW_IGNITER_OFF (HAL_GPIO_ReadPin(SW_IGNITER_GPIO_Port, SW_IGNITER_Pin))==GPIO_PIN_RESET

#define SW_YAW_R_OFF (HAL_GPIO_ReadPin(SW_YAW_R_GPIO_Port, SW_YAW_R_Pin))==GPIO_PIN_RESET
#define SW_YAW_L_OFF (HAL_GPIO_ReadPin(SW_YAW_L_GPIO_Port, SW_YAW_L_Pin))==GPIO_PIN_RESET

// 舵机宏
//PA8
#define servo_igniter_unlock    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1,servo_ccr.igniter_ccr_unlock ) // 扳机舵机解锁      ,120卡住,170ok
#define servo_igniter_lock      __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1,servo_ccr.igniter_ccr_lock ) // 扳机舵机锁止

//装填舵机即升降机左右的舵机，上为升，下为下降，下降即装填飞镖，上升即清空发射区
//PB6
#define servo_loader_up1     __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, servo_ccr.loader1_ccr_up)   // 装填舵机左，上升
#define servo_loader_down1   __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, servo_ccr.loader1_ccr_down)  // 装调舵机左，下降
//PB7
#define servo_loader_up2     __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, servo_ccr.loader2_ccr_up)  // 装填舵机右，上升
#define servo_loader_down2   __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, servo_ccr.loader2_ccr_down)  // 装填舵机右，下降

//转移舵机即动作舱储存区的卡镖舵机，负责将新镖从储存区转移到发射区
//PA11
#define servo_transfomer_lock     __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, servo_ccr.transfomer_ccr_lock)  // 卡镖舵机维持卡镖
#define servo_transfomer_unlock   __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, servo_ccr.transfomer_ccr_unlock)  // 卡镖舵机松开卡镖

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

#define MASK_DELIVER_L_CALIBRATED (1 << 0)
#define MASK_DELIVER_R_CALIBRATED (1 << 1)
#define MASK_IGNITER_CALIBRATED (1 << 2)
#define MASK_YAW_L_CALIBRATED (1 << 3)
#define MASK_YAW_R_CALIBRATED (1 << 4)

#define MASK_ALL_CALIBRATED (MASK_DELIVER_L_CALIBRATED | MASK_DELIVER_R_CALIBRATED | MASK_IGNITER_CALIBRATED | MASK_YAW_L_CALIBRATED | MASK_YAW_R_CALIBRATED)

#define enum_X_Macros_disable 0

#ifdef __cplusplus
}
#endif
