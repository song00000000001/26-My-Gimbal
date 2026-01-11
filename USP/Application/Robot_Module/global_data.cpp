#include "global_data.h"
#include "robot_config.h"
#include "internal.h"


//视觉通信协议用,aiming_controller.cpp中会读取。
//communication.cpp中在硬件串口中断拷贝更新,在service_deice.cpp任务中，和摇杆联动进行视觉控制，有标定等任务，
VisionRecvData_t vision_recv_pack;//电视接收包
VisionSendData_t vision_send_pack;//电视发送包
uint32_t vision_last_recv_time = 0; // 视觉最后接收时间,communication.cpp中更新,server发送时使用
/*todo
song
1.确认视觉数据结构体内容
2.确认数据更新频率
3.确认视觉标定过程
*/

//调参板数据定义
// 打击目标
DartAimEnumdef HitTarget; 
//发射数据池,每组数据包含一次发射(包含两种目标)的yaw和igniter目标数据,总共是4个浮点数
DartDataStructdef DartsData[16]; 
//发射数据映射表,调参板会调整这个表,而本地使用这个表来在发射数据池选择哪一组数据
uint8_t DartDataSlot[5]={0,1,2,3,4};

// 实例化驱动,发射类在发射主控任务中使用,service_communication.cpp中更新电机数据
Launcher_Driver Launcher(ID_DELIVER_L, ID_DELIVER_R, ID_IGNITER); 
// yaw控制类实例化,yaw_control.cpp等任务中使用,service_communication.cpp中更新电机数据
Missle_YawController_Classdef Yawer(ID_YAW);
// 定义全局机器人状态 (Cmd, Flag, Status)
Robot_Ctrl_t Robot;
//校准速度结构体实例化
calibration_speed_t calibration_speed;
//调试数据结构体实例化
Debug_Data_t Debugger;
//遥控器数据快照
DR16_Snapshot_t DR16_Snap; 

/*舵机调参记录
撒放舵机从松开到锁定
A8-igniter:170~270

升降机从下到上，舵机运动范围
B7左舵机loader2:235~100
B6右舵机loader1:75~190

卡镖从卡锁到松开:
A11-transfomer:126~170

*/

servo_ccr_debug servo_ccr={
    170,    //igniter_ccr_unlock
    270,    //igniter_ccr_lock

    190,    //loader1_ccr_up
    75,     //loader1_ccr_down

    100,    //loader2_ccr_up
    235,    //loader2_ccr_down

    205,    //transfomer_ccr_lock
    180     //transfomer_ccr_unlock
};
