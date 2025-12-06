#include "global_data.h"
#include "robot_config.h"
#include "internal.h"

//主控制任务类实例化,launchest.cpp等任务中使用,service_communication.cpp中更新电机数据
Motor_GM6020 loadermotor[1]{ID_LOADER};//装填电机
Launch_Classdef Launch(ID_DELIVER_R, ID_DELIVER_L, ID_IGNITER); //发射类
Missle_YawController_Classdef Yaw(ID_YAW);//yaw控制类

//视觉通信协议用,aiming_controller.cpp中会读取。
//communication.cpp中在硬件串口中断拷贝更新,
//在service_deice.cpp任务中，和摇杆联动进行视觉控制，有标定等任务，
/*todo
song
1.确认视觉数据结构体内容
2.确认数据更新频率
3.确认视觉标定过程
*/
VisionRecvData_t vision_recv_pack;//电视接收包
VisionSendData_t vision_send_pack;//电视发送包
uint32_t vision_last_recv_time = 0; // 视觉最后接收时间,communication.cpp中更新,server发送时使用

//调参板通信协议用
//aiming_controller.cpp中修改,protocol.cpp中使用
DartAimEnumdef HitTarget; //打击目标,默认打前哨站,
