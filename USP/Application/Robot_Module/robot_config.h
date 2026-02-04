#pragma once
#include "SRML.h"
#include "tim.h"    

#ifdef __cplusplus
extern "C" {
#endif

//保守测试参数切换宏
#define CONSERVATIVE_TEST_PARAMS 0

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

#define DELIVER_OFFSET_POS   -20.0f  // deliver碰到开关后设置的初始坐标
#define IGNITER_OFFSET_POS   3.0f   // igniter复位位置
														                                        
#define POS_BUFFER -20.0f          //缓冲区位置
#define POS_BOTTOM -647.0f         //拉栓位置
			
#define POS_IGNITER 90.0f         //默认力度,igniter位置

//igniter最小/大位置
#define IGNITER_MIN_POS 2.0f
#define IGNITER_MAX_POS 200.0f
//deliver最小/大位置
#define POS_DELIVER_MIN -647.0f
#define POS_DELIVER_MAX -5.0f
//yaw最小/大位置,警告: 如果不清楚该值的映射关系,请勿随意更改!谨慎修改!
#define YAW_MIN_ANGLE -10.2f
#define YAW_MAX_ANGLE 10.2f

//手动控制速度
#define YAW_MANUAL_SPEED 0.02f //手动控制时的yaw轴速度设定值
#define IGNITER_MANUAL_SPEED 0.04f //手动控制时的igniter轴速度设定值

#define YAW_VISION_PID_OUTPUT_SCALE 1000 //视觉pid输出缩放因子

//到位阈值
#define IGNITER_ARRIVE_THRESHOLD 5.0f
#define DELIVER_ARRIVE_THRESHOLD 5.0f
#define YAW_ARRIVE_THRESHOLD 5.0f  
#define YAW_VISION_STABLE_THRESHOLD 0.05f //视觉瞄准稳定阈值

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
