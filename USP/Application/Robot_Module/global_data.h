#pragma once

#include "stdint.h"
#include "launchest.h"
#include "Yaw_control.h"

// 声明全局机构对象
extern Launch_Classdef Launch; // 发射类
extern Missle_YawController_Classdef Yaw; // yaw控制类

// 声明任务句柄


// 调试/通信用变量


// 飞镖数据存储相关定义
//最大飞镖数据池大小?待确认
#define MAX_DART_DATAPOOL_SIZE 16
//飞镖数据结构体,用于选择目标是基地还是前哨站
typedef enum __DartAimEnumdef
{
  Outpost = 0,
  Base = 1
}DartAimEnumdef;
//飞镖数据结构体,用于存储飞镖目标数据
typedef struct __DartDataStructdef
{
  double Ignitergoal[2]; //扳机目标位置
  double YawCorrectionAngle[2];//偏航修正角
}DartDataStructdef;


extern DartDataStructdef DartsData[MAX_DART_DATAPOOL_SIZE]; // 飞镖数据
extern uint8_t DartDataSlot[5];                             // 发射数据选择
extern DartAimEnumdef HitTarget;                            // 打击目标
extern uint8_t ParamSendPack[9];
extern uint8_t CurrentCnt; // 当前发数
extern float Yaw_Angle[2]; // 默认前哨站和基地角度



/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/

