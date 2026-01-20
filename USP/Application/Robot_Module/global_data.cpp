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
uint8_t DartDataSlot[4]={1,2,3,4};

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
    250,    //igniter_ccr_unlock
    320,    //igniter_ccr_lock

    215,    //loader1_ccr_up
    75,     //loader1_ccr_down

	
    95,    //loader2_ccr_up
    230,    //loader2_ccr_down

    235,    //transfomer_ccr_lock
    180     //transfomer_ccr_unlock
};

protocol_status_t Protocol_Status[4]={
    {false,0,100,0.0f,0.5f}, //CAN1,MOTOR_DELIVER_L
    {false,0,100,0.0f,0.5f}, //CAN1,MOTOR_DELIVER_R
    {false,0,100,0.0f,0.5f}, //CAN1,MOTOR_IGNITER
    {false,0,100,0.0f,0.5f}  //CAN2,MOTOR_YAW
};




/*发射流程延时参数说明
put_delay
指滑块到底部后，升降机从动态同步到平行位置的等待缓冲时间，同时也让滑台能卡上扳机，防止滑台来回太快扳机因为摩擦力没弹回去导致没扣住。给300应该够。
before_fire_delay
指滑块在升降机下方经过并回到缓冲区后，升降机从平行位置降到底和镖体分离的缓冲时间。
after_fire_delay
指发射后等待时间，单位ms，是为了给发射动作留时间，防止发射到一半滑块又回去相撞。制导镖200mm行程发射时间为133ms,1000ms足够保险。
relapse_delay
指卡镖舵机松开后等待时间，单位ms，是为了给镖体转移到发射区留时间，防止舵机太快锁住卡不住。测试时带制导镖完全转移时间为139ms。但是给100ms也没问题。完全放的掉。
loader_up_delay指升降机上升到顶部的等待时间，单位ms，是为了给升降机上升留时间，防止还没上去卡镖舵机就动作了。带制导镖上升时间为667ms。
*/

#if CONSERVATIVE_TEST_PARAMS
// 使用保守测试参数
fire_sequence_delay_params_t fire_sequence_delay_params={
    1000,    //put_delay
    1000,    //before_fire_delay
    1000,    //after_fire_delay
    300,     //relapse_delay
    1000,     //loader_up_delay
    1000     //wait_for_aim_delay
};
#else
fire_sequence_delay_params_t fire_sequence_delay_params={
    300,    //put_delay
    500,    //before_fire_delay
    500,    //after_fire_delay
    300,    //relapse_delay
    700,     //loader_up_delay
    500     //wait_for_aim_delay
};
#endif