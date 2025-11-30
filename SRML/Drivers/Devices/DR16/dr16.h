/**
  ******************************************************************************
  * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
  * @file    dr16.h
  * @author  Zelong.Xu 8762322@qq.com
  * @brief   Code for DJI-DR16 driver in embedded software system.
  ******************************************************************************
  * @attention
  *
  * if you had modified this file, please make sure your code does not have any
  * bugs, update the version Number, write dowm your name and the date. The most
  * important thing is make sure the users will have clear and definite under-
  * standing through your new brief.
  *
  * <h2><center>&copy; Copyright (c) 2019 - ~, SCUT-RobotLab Development Team.
  * All rights reserved.</center></h2>
  ******************************************************************************
  */
#ifndef _DR16_H_
#define _DR16_H_

#ifdef __cplusplus
/* Includes ------------------------------------------------------------------*/
#include "srml_std_lib.h"
#include "Drivers/Devices/Remote_Public.h"
/* Private macros ------------------------------------------------------------*/
/* 键位定义 */
enum
{
  DR16_KEY_W = 0,
  DR16_KEY_S,
  DR16_KEY_A,
  DR16_KEY_D,
  DR16_KEY_SHIFT,
  DR16_KEY_CTRL,
  DR16_KEY_Q,
  DR16_KEY_E,
  DR16_KEY_R,
  DR16_KEY_F,
  DR16_KEY_G,
  DR16_KEY_Z,
  DR16_KEY_X,
  DR16_KEY_C,
  DR16_KEY_V,
  DR16_KEY_B,
  DR16_MOUSE_L,
  DR16_MOUSE_R,
};

/* Private type --------------------------------------------------------------*/
/** 
  @brief DR16数据包内容
*/
#pragma pack(1)
struct DR16_DataPack_Typedef
{
  uint64_t ch0:11;
  uint64_t ch1:11;
  uint64_t ch2:11;
  uint64_t ch3:11;
  uint64_t s2:2;
  uint64_t s1:2;
  int64_t x:16;
  int64_t y:16;
  int64_t z:16;
  uint64_t press_l:8;
  uint64_t press_r:8;
  uint64_t key:16;
};
#pragma pack()

/**
  @brief 按键类型定义
*/
struct Key_Typedef
{
    bool isPressed;   /*<! 是否按下 */
    bool isTriggered; /*<! 是否触发过函数，用来执行点击事件 */
};

typedef void(*CLICK_EXCE)(void);  /*<! 单击执行函数 */
/* Exported macros -----------------------------------------------------------*/
#define Ignore_Limit 0.05         /*<! 线性死区,手柄或鼠标的归一化后的绝对值小于此值时自动视为0 */

// 兼容老式enum命名
#define DR16_ESTABLISHED  ESTABLISHED 
#define DR16_LOST         LOST 
#define DR16_SW_NONE      SW_NONE
#define DR16_SW_UP        SW_UP
#define DR16_SW_MID       SW_MID
#define DR16_SW_DOWN      SW_DOWN
/* Exported types ------------------------------------------------------------*/
/* DR16类型 */
class DR16_Classdef
{
    private:
        uint32_t dt = 0;
        LinkageStatus_Typedef Status;   /*<! 连接状态 */
        uint8_t  detecting_flag;        /*<! 检测操作标志位*/
		uint32_t last_check_time;       /*<! 上一次在线检测时间*/
        DR16_DataPack_Typedef DataPack; /*<! 数据包*/
        float RX_Norm,RY_Norm,LX_Norm,LY_Norm,MouseX_Norm,MouseY_Norm,MouseZ_Norm;
                                        /*<! 两个摇杆四个方向与鼠标三个方向速度归一化后的值*/
        Key_Typedef Key[18];            /*<! 16个键的相关信息*/
		CLICK_EXCE Click_Fun[18];       /*<! 单击执行函数 */
        float MouseCoefficient;         /*<! 鼠标动作乘的系数*/
        void Key_Process(void);         /*<! 按键处理*/

    public:
/* Exported function declarations --------------------------------------------*/
        DR16_Classdef();
        void DataCapture(DR16_DataPack_Typedef* captureData);

        uint64_t GetCh0(void);
        uint64_t GetCh1(void);
        uint64_t GetCh2(void);
        uint64_t GetCh3(void);
        SW_Status_Typedef GetS2(void);
        SW_Status_Typedef GetS1(void);
        int64_t GetMouseX(void);
        int64_t GetMouseY(void);
        int64_t GetMouseZ(void);
        uint64_t GetPress_L(void);
        uint64_t GetPress_R(void);
        uint64_t Getkey(void);

        /*归一化后的通道0123、鼠标XYZ值*/
        float Get_RX_Norm(void);
        float Get_RY_Norm(void);
        float Get_LX_Norm(void);
        float Get_LY_Norm(void);
        float Get_MouseX_Norm(void);
        float Get_MouseY_Norm(void);
        float Get_MouseZ_Norm(void);

        /*用于判断某个按键是否按下,按下之后的回调函数*/
        bool IsKeyPress(int _key);
        void Register_Click_Fun(int _Key, CLICK_EXCE Fun_Ptr);
        void Exce_Click_Fun();

        /*连接状态相关操作*/
        void Check_Link(uint32_t current_check_time);
		    void SetStatus(LinkageStatus_Typedef para_status);
        LinkageStatus_Typedef GetStatus(void);
};

#endif
#endif
/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
