#include "internal.h"
#include "global_data.h"
#include "launcher_driver.h"
#include "robot_config.h"

/*摇杆逻辑
markdown:

| 摇杆  | 状态     | 动作                                                                 |     | 摇杆  | 状态         | 动作                             |
| --- | ------ | ------------------------------------------------------------------ | --- | --- | ---------- | ------------------------------ |
| s1  | up     | 电机失能（不修改状态）                                                        |     | s2  | up         | 手动控制yaw和igniter                |
|     | middle | 进自检，自检手动完成或者跳过自动进校准，校准完后进等待发射，（卡stay状态，后续的autofire状态会因此被重置到stay状态） |     |     | middle     | 使用修正值控制yaw和igniter（修正值由调参板设置）  |
|     | down   | 激活自动发射，（进autofireprep状态），跳过自检（自检就是手动按限位开关）                         |     |     | down       | 使用视觉控制，如果视觉失联，转而用修正值控制，直到视觉重连。 |
| LX  |        |                                                                    |     | RX  | left/right | 增量控制yaw                        |
| LY  |        |                                                                    |     | RY  | up/donw    | 增量控制igniter                    |
s1要是多个状态就可以在自检完后进idle空闲而不是直接开始校准（电机会直接开转），不然就要事先在自检前失能，检完后使能（middle)，然后打down自动发射。
todo:
1. 改发射条件,需要手动掰LY到底发射,并且向左向右选择连发模式2/4发。连发模式还没写。可以写在发射机里。
2.

date:2025/12/10/0:47
*/



/*发射主控任务*/
void LaunchCtrl(void *arg)
{
    //can发送的包
    Motor_CAN_COB Tx_Buff,Tx_Buff1;
 
    // 初始状态设为自检
    Robot.Status.current_state = SYS_CHECKING;
    // 初始自检标志位失能
	Robot.Flag.Check.limit_sw_ok=false;
    //跳过自检标志位失能,只会在s1下时生效,并且会在s1为中重置
    Robot.Cmd.skip_check=false;

    // Debug 初始化
    Debugger={
        .enable_debug_mode=false,
        .debug_mode_deliver={MODE_SPEED,MODE_SPEED},
        .debug_mode_igniter=MODE_SPEED ,
        .stall_params_deliver={8000,10,500},//堵转参数初始化，limit_output,speed_threshold,time_ms
        .stall_params_igniter={4000,10,500},
        .stall_params_yaw={8000,5,500}
    };

    //校准速度初始化
    calibration_speed={
	.yaw_calibration_speed=-300,
	.deliver_calibration_speed=600,
    .igniter_calibration_speed=-800
    };

    // 任务频率控制
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1);
    for (;;)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
		
        if (DR16.GetStatus() != DR16_ESTABLISHED) {
            Robot.Flag.Status.rc_connected = false;
            Robot.Status.current_state = SYS_OFFLINE;
        }
        else{
            Robot.Flag.Status.rc_connected = true;
            // Debug 模式判定 (最高优先级的主动模式)
            // 只有当遥控器连接，且全局 Debug 标志位被置 1 时进入，并且校准完成。
            if (Debugger.enable_debug_mode&&Robot.Flag.Status.is_calibrated) {
                Robot.Status.current_state = SYS_DEBUG;
            }

            // 处理遥控器开关逻辑
            // 手动失能开关 S1向上
            if(DR16.GetS1()==SW_UP){
                Robot.Cmd.sys_enable=false;
            }
            else{
                Robot.Cmd.sys_enable=true;
            }
            //自动发射模式,跳过自检 S1向下
            if(DR16.GetS1()==SW_DOWN){
                Robot.Cmd.skip_check=true;
                Robot.Cmd.auto_mode=true;
            }
            //s1中则是自动发射关闭,会卡在待机状态,并且不会跳过自检
            if(DR16.GetS1()==SW_MID){
                Robot.Cmd.auto_mode=false;
                Robot.Cmd.skip_check=false;
            }
            //s2
            if(DR16.GetS2()==SW_UP){
                //Robot.Status.yaw_control_state = MANUAL_AIM; //切换到手动瞄准
                //这里无需切换,保持即可,不然会抢占校准状态,校准完后,主状态机自动切换到手动模式
            }
            else if(DR16.GetS2()==SW_MID){
                Robot.Status.yaw_control_state = CORRECT_AIM; //切换到修正值瞄准
            }
            else{
                Robot.Status.yaw_control_state = VISION_AIM; //切换到视觉瞄准
            }
            /*todo
            song
            修改所有调用dr16数据的地方为快照数据。
            目前就只有yaw的手动微调用了
            */

            DR16_Snap.LX_Norm=DR16.Get_LX_Norm();
            DR16_Snap.LY_Norm=DR16.Get_LY_Norm();
            DR16_Snap.RX_Norm=DR16.Get_RX_Norm();
            DR16_Snap.RY_Norm=DR16.Get_RY_Norm();
            DR16_Snap.S1=DR16.GetS1();
            DR16_Snap.S2=DR16.GetS2();
            DR16_Snap.Status=DR16.GetStatus();
        }
        /*在非校准状态如果发生碰撞限位的现象，则立即取消使能并且记录错误电机信息，
        并且重置为error状态，此时会将该电机的校准状态重置,  
        需要重新进行限位校准，此时如果拨右摇杆朝下则会反向旋转对应电机，
        拨左摇杆取消使能则进入check状态，
        */

        switch (Robot.Status.current_state)
        {
        case SYS_DEBUG:
        {
             // 在 Debug 模式下：
            // 1. 不执行任何自动逻辑 (fire sequence等)
            // 2. adjust() 依然运行，但 target 不会被代码修改
            // 3. 用户在 Watch 窗口直接修改 Launcher.pid_xxx.Target 或 Kp Ki Kd

            //利用debug结构体修改电机模式
            Launcher.mode_deliver[0]=Debugger.debug_mode_deliver[0];
            Launcher.mode_deliver[1]=Debugger.debug_mode_deliver[1];
            Launcher.mode_igniter=Debugger.debug_mode_igniter;
            
            // 如果手动失能，则回 Checking 状态
            if (Robot.Status.current_state == SYS_DEBUG && !Robot.Cmd.sys_enable) {
                // 安全起见重新自检
                Launcher.check_progress=0; // 重置自检进度
                Robot.Status.current_state = SYS_CHECKING;
                //为了防止自己跳过自检（电机速度角度环状态只在校准后才会切换角度环，而debug中可能会改成速度环然后退出，那么后续就不会进入角度环模式，那就不行）
                Launcher.mode_deliver[0] = MODE_ANGLE;
                Launcher.mode_deliver[1] = MODE_ANGLE;
                Launcher.mode_igniter = MODE_ANGLE;
                // 重置目标值为当前值，防止猛冲
                Launcher.target_deliver_angle = Launcher.DeliverMotor[0].getMotorTotalAngle(); 
                Launcher.target_igniter_angle = Launcher.IgniterMotor.getMotorTotalAngle();
            }
            
        }
        break;
			
        case SYS_OFFLINE:
        {
            // 恢复条件：遥控器重连
            if (Robot.Flag.Status.rc_connected) {
                Robot.Status.current_state = SYS_CHECKING;
            }
        }
        break;
        case SYS_CHECKING:
        {
            //按键自检逻辑
            Launcher.key_check();
            // 3. 处理跳过逻辑
            if (Robot.Cmd.skip_check) {
                Launcher.check_progress = MASK_ALL_PASSED; // 强制全满
            }

            // 4. 更新全局标志位
            Robot.Flag.Check.limit_sw_ok = (Launcher.check_progress == MASK_ALL_PASSED);
   
            // 5个按键手动检查全部通过则进入校准状态,后续可以加入电机检查
            if (Robot.Flag.Check.limit_sw_ok) {
                Robot.Status.current_state = SYS_CALIBRATING;
                //注意,这里启动了校准过程,会配置电机为速度环,直到撞到限位开关
                Launcher.start_calibration();
            }
        }
        break;
        
        case SYS_CALIBRATING:
            //calibration_speed;
            // 此状态下，Launcher.adjust() 内部正在跑归零逻辑，任务层只需要等待驱动层反馈 "已校准"
            //yaw控制考虑到是并行的，主状态机和子状态机采用状态位判断
            Robot.Status.yaw_control_state = YAW_CALIBRATING;
            if(Yawer.is_Yaw_Init()){
                Robot.Status.yaw_control_state = MANUAL_AIM; //校准完成后，进入手动模式
                Yawer.mode_YAW = MODE_ANGLE; //切换回角度环
                Yawer.yaw_target=0;
            }
            if(Launcher.is_calibrated()){
                Launcher.target_igniter_angle=POS_IGNITER;  // 默认力度
                Launcher.target_deliver_angle=POS_BUFFER;   // 回缓冲
            }
            // 1. 处理归零状态转换
            Launcher.check_calibration_logic();
            Robot.Flag.Status.is_calibrated=Yawer.is_Yaw_Init()&&Launcher.is_calibrated();
            // 2. 全部校准完毕后，切换到待机状态
            //校准完毕跳转待机状态
            if (Robot.Flag.Status.is_calibrated) {
                Robot.Status.current_state = SYS_CALIBRATED;
            }
            break;
        //校准完毕时，总有一个限位开关被触发，防止误触发限位开关进入error状态
        //因此加一个校准完毕后给角度环到安全位置的状态停留，直到都到达指定位置后再进入待机状态
        case SYS_CALIBRATED:
        {
            Launcher.target_igniter_angle=POS_IGNITER;  // 默认力度
            Launcher.target_deliver_angle=POS_BUFFER;   // 回缓冲
            Yawer.yaw_target=0;
            if(Launcher.is_deliver_at_target()&&Launcher.is_igniter_at_target()&&Yawer.isMotorAngleReached(5.0f))
            {
                Robot.Status.current_state = SYS_STANDBY;
            }

        }
            break;

        case SYS_STANDBY:
            // 切换到自动模式
            if (Robot.Cmd.auto_mode) {
                Launcher.fire_state = FIRE_IDLE; // 重置发射子状态机
                Robot.Status.current_state = SYS_AUTO_PREP;
            }
            break;
            
        case SYS_AUTO_PREP:
            // --- 自动发射准备 ---
            //确保机构归位到待发状态
			Launcher.target_deliver_angle=POS_BUFFER;   // 回缓冲

            // 检查是否到位
            if (Launcher.is_deliver_at_target() && Launcher.is_igniter_at_target()) 
            {  
                Robot.Status.current_state = SYS_AUTO_FIRE;
            }
            
            // 随时允许切回手动
            if (!Robot.Cmd.auto_mode) Robot.Status.current_state = SYS_STANDBY;
            break;
            
        case SYS_AUTO_FIRE:
            Launcher.Run_Firing_Sequence();
            // 随时允许切回手动 (Run_Firing_Sequence 内部也会处理打断复位)
            if (!Robot.Cmd.auto_mode) Robot.Status.current_state = SYS_STANDBY;
            break;
        }

        //yaw轴子状态机,包含状态如下
        /*
            manual_aim:手动瞄准
            vision_aim:视觉瞄准
            correct_aim:修正值瞄准
            disable_motor:失能电机
            yaw_calibrating:校准模式
        */
        Yawer.yaw_state_machine(Robot.Status.yaw_control_state, DR16_Snap.LX_Norm, DR16_Snap.LY_Norm);

        /*todo
        song
        堵转保护后进入校准会导致滑块电机装上限位开关停不下来，后续排查清楚原因再引入，先注释掉
        */
        // 2. 堵转保护 (全局生效，除 Debug/Offline)
        #if 0
        if (Robot.Status.current_state != SYS_OFFLINE  
            && Robot.Status.current_state != SYS_ERROR 
            //&& Robot.Status.current_state != SYS_DEBUG  // Debug时可能会手动扭角度环。
        ) 
        {
            // 如果 PID 输出超过 8000 (约50%力矩) 且速度小于10，持续1000ms -> 错误状态
            bool stall_detected = false;
            /*todo
            后期可能在发射子状态机的某个状态不检测滑块堵转，但前期检测它最容易，所以先全局用着。
            不检测的原因是滑块会有一个状态是拉到底然后卡住等待装填，这个时候速度肯定是0，如果检测堵转就会误判。
            倒时候也可以在发射子状态机里处理这个逻辑，比如说在等待装填状态不检测堵转。
            */
            stall_detected |=Launcher.check_deliver_stall(
                Debugger.stall_params_deliver.limit_output,
                Debugger.stall_params_deliver.threhold_rpm,
                Debugger.stall_params_deliver.time_ms);
            //丝杆堵转检测
            stall_detected |= Launcher.check_igniter_stall(
                Debugger.stall_params_igniter.limit_output,
                Debugger.stall_params_igniter.threhold_rpm, 
                Debugger.stall_params_igniter.time_ms);
            //yaw轴堵转检测
            stall_detected |= Yawer.yaw_stall_check(
                Debugger.stall_params_yaw.limit_output,
                Debugger.stall_params_yaw.threhold_rpm,
                Debugger.stall_params_yaw.time_ms);
            
            if (stall_detected) 
            {
                Robot.Status.current_state = SYS_ERROR;
                //防止卡死在堵转保护状态，需要将pid目标重置为当前值，并且清空输出
                Launcher.target_deliver_angle = Launcher.DeliverMotor[0].getMotorTotalAngle();
                Launcher.target_deliver_angle = Launcher.DeliverMotor[1].getMotorTotalAngle();
                Launcher.target_igniter_angle = Launcher.IgniterMotor.getMotorTotalAngle();
                Yawer.yaw_target = Yawer.YawMotor.getMotorTotalAngle();
                Launcher.pid_deliver_spd[0].Target=0;
                Launcher.pid_deliver_spd[1].Target=0;
                Launcher.pid_igniter_spd.Target=0;
                Yawer.PID_Yaw_Speed.Target=0;
                //计算 PID,使输出为0
                Launcher.adjust();
                Yawer.adjust();
                Launcher.pid_deliver_spd[0].Out=0;
                Launcher.pid_deliver_spd[1].Out=0;
                Launcher.pid_igniter_spd.Out=0;
                Yawer.PID_Yaw_Speed.Out=0;
                //Launcher.clear_all_motor_output();
            }
        }
        #endif
		 //计算 PID
		Launcher.adjust();
		Yawer.adjust();
			
        if (Robot.Status.current_state != SYS_OFFLINE && 
            Robot.Status.current_state != SYS_CHECKING&&
            Robot.Cmd.sys_enable
        ) 
        {
           
            //输出电流
            Launcher.out_all_motor_speed();
            Yawer.yaw_out_motor_speed();
        }
        else
        {
            //停止电机
            Launcher.stop_all_motor();
            Yawer.disable();
        }
        
        MotorMsgPack(Tx_Buff1, Yawer.YawMotor);
        xQueueSend(CAN2_TxPort, &Tx_Buff1.Id1ff, 0);
        //R0L1
        MotorMsgPack(Tx_Buff, Launcher.DeliverMotor[1], Launcher.DeliverMotor[0], Launcher.IgniterMotor);
		xQueueSend(CAN1_TxPort, &Tx_Buff.Id200, 0);

    }
}


