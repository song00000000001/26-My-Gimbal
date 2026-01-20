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
inline const char* Get_SW_Str(uint8_t sw) {
    switch (sw) {
        case SW_UP:   return "UP";
        case SW_MID:  return "MID";
        case SW_DOWN: return "DOWN";
        default:      return "NONE";
    }
}

/*发射主控任务*/
void LaunchCtrl(void *arg)
{
    //can发送的包
    Motor_CAN_COB Tx_Buff,Tx_Buff1;

    //初始状态设为离线
    Robot.Status.current_state=SYS_DISCONNECTED;
    // 重置自检进度
    Launcher.check_progress=0; 
    //yaw轴控制状态初始化为失能
    Robot.Status.yaw_control_state=YAW_MANUAL_AIM;
    //初始化飞镖发射数量
    Robot.Status.dart_count=1;
    //初始自检限位开关标志位失能
	Robot.Flag.Check.limit_sw_ok=false;
    //初始校准标志位失能
    Robot.Flag.Status.is_calibrated=false;
    //初始化遥控连接失能
    Robot.Flag.Status.rc_connected=false;
    //初始化停止连发标志位失能
    Robot.Flag.Status.stop_continus_fire=false;
    //跳过自检标志位失能,只会在s1下时生效,并且会在s1为中重置
    Robot.Cmd.skip_check=false;
    //初始自动发射模式标志位失能
    Robot.Cmd.autofire_enable=false;
    //初始发射指令标志位失能
    Robot.Cmd.fire_command=false;
    //初始化系统使能标志位失能
    Robot.Cmd.sys_enable=false;
 
    // Debug 初始化,用于控制电机状态
    Debugger={
        .enable_debug_mode=0,//用于debug中进入debug状态
        .debug_mode_deliver={MODE_SPEED,MODE_SPEED},
        .debug_mode_igniter=MODE_SPEED ,
        .debug_loader_pos=POS_BOTTOM,
        .debug_fire_type=3, //调整发射类型，0为连发一二三四，1为单发第一发，2为单发第二发，3为单发第三发。  
        .is_loader_simulating=false,
        .simulated_loader_pos=-650.0f,
        .four_dart_four_params_enable=false,//四发四参功能启用标志位，默认禁用，调试中启用。
        .dual_loader_mechanical_error_correction=3.0,//双滑块机械装配误差校准修正,目前靠0号,即发射方向左滑块减2.2mm(向下)解决。
        .deliver_sync_threshold=0.5 //滑块同步误差阈值
    };
    #if CONSERVATIVE_TEST_PARAMS
    //校准速度初始化
    calibration_speed={
	.yaw_calibration_speed=-300,
	.deliver_calibration_speed=600,
    .igniter_calibration_speed=-600
    };
    // PID 参数初始化
    Launcher.pid_deliver_sync.SetPIDParam(-1.0f, 0.0f, 0.0f, 8000, 10000);
    
    for(int i=0; i<2; i++) {
        Launcher.pid_deliver_spd[i].SetPIDParam(20.0f, 2.0f, 0.0f, 8000, 14000);
        Launcher.pid_deliver_pos[i].SetPIDParam(800.f, 0.0, 0.0, 1000, 6000);
    }
    
    Launcher.pid_igniter_spd.SetPIDParam(15.0, 0.0, 0.0, 3000, 8000);
    Launcher.pid_igniter_pos.SetPIDParam(3000.0, 0.0, 0.0, 3000, 4000);

    Yawer.PID_Yaw_Angle.SetPIDParam(15, 0, 0, 0, 200);
    Yawer.PID_Yaw_Angle.I_SeparThresh = 8;
    Yawer.PID_Yaw_Angle.DeadZone = 0.01f;
    Yawer.PID_Yaw_Speed.SetPIDParam(20, 0, 0, 0, 12000);
    #else
	//校准速度初始化
    calibration_speed={
	.yaw_calibration_speed=-600,
	.deliver_calibration_speed=1200,
    .igniter_calibration_speed=-1200
    };
    // PID 参数初始化
    Launcher.pid_deliver_sync.SetPIDParam(-0.8f, 0.0f, 0.0f, 8000, 16000);
    
    for(int i=0; i<2; i++) {
        Launcher.pid_deliver_spd[i].SetPIDParam(20.0f, 2.0f, 0.0f, 8000, 16384);
        Launcher.pid_deliver_pos[i].SetPIDParam(800.f, 0.0, 0.0, 1000, 10000);
    }
    
    Launcher.pid_igniter_spd.SetPIDParam(15.0, 0.0, 0.0, 3000, 12000);
    Launcher.pid_igniter_pos.SetPIDParam(3000.0, 0.0, 0.0, 3000, 6000);

    Yawer.PID_Yaw_Angle.SetPIDParam(15, 0, 0, 0, 300);
    Yawer.PID_Yaw_Angle.I_SeparThresh = 8;
    Yawer.PID_Yaw_Angle.DeadZone = 0.01f;
    Yawer.PID_Yaw_Speed.SetPIDParam(20, 0, 0, 0, 18000);
	#endif
    
    // 任务频率控制
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1);
    uint32_t main_task_now = xTaskGetTickCount();

	LOG_INFO("Launch Control Task Started.\n");
    #if enum_X_Macros_disable
    LOG_INFO("Initial System State: %d,yaw_control_state: %d,loader_target_mode: %d\n", 
        Robot.Status.current_state, Robot.Status.yaw_control_state, Launcher.loader_target_mode);
    #else
    LOG_INFO("Initial System State: %s,yaw_control_state: %s,loader_target_mode: %s\n", 
        System_State_To_Str(Robot.Status.current_state),
        Yaw_Control_State_To_Str(Robot.Status.yaw_control_state),
        Loader_Target_Mode_To_Str(Launcher.loader_target_mode));
    #endif

    for (;;)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        main_task_now = xTaskGetTickCount();

        // 1. 更新遥控器快照数据 & 处理遥控器逻辑
        /*todo
        song
        互斥锁需要测试。
        主任务不能被阻塞。
        */
        // 尝试拿锁，参数为 0 表示：拿不到立刻返回，不等待，不阻塞
        if (xSemaphoreTake(DR16_mutex, 0) == pdTRUE)
        {
            // 1. 拿到了锁：更新快照
            DR16_Snap.Status = DR16.GetStatus();
            DR16_Snap.LX_Norm = DR16.Get_LX_Norm();
            DR16_Snap.LY_Norm = DR16.Get_LY_Norm();
            DR16_Snap.RX_Norm = DR16.Get_RX_Norm();
            DR16_Snap.RY_Norm = DR16.Get_RY_Norm();
            DR16_Snap.S1 = DR16.GetS1();
            DR16_Snap.S2 = DR16.GetS2();
            
            // 2. 释放锁
            xSemaphoreGive(DR16_mutex);
        }

        // 记录遥控器指令变化
        /*todo
        song
        可能会影响性能,需要测试。
        也可以只在状态变化时打印。
        */
		#if 0	
        static DR16_Snapshot_t last_DR16_Snap = DR16_Snap;
        if(memcmp(&last_DR16_Snap, &DR16_Snap, sizeof(DR16_Snapshot_t)) != 0) {
            LOG_INFO("DR16 Updated: S1=%d, S2=%d, LX=%.2f, LY=%.2f, RX=%.2f, RY=%.2f", 
                DR16_Snap.S1, DR16_Snap.S2, DR16_Snap.LX_Norm, DR16_Snap.LY_Norm, DR16_Snap.RX_Norm, DR16_Snap.RY_Norm);
            memcpy(&last_DR16_Snap, &DR16_Snap, sizeof(DR16_Snapshot_t));
        }
		#else
        //由于上面打印信息太多,改为只打印开关变化
		static uint8_t last_S1 = DR16_Snap.S1;
        static uint8_t last_S2 = DR16_Snap.S2;
        if (last_S1 != DR16_Snap.S1) {
            LOG_INFO("DR16 Switch Updated: S1=%s -> %s", 
                Get_SW_Str(last_S1), Get_SW_Str(DR16_Snap.S1));
            last_S1 = DR16_Snap.S1;
            /*
            日志增加手动触发失能和暂停时，记录各电机的运动位置，用于判断当时是否出现电机角度脱离软件限位的情况（即撞了限位开关。）以及卡死时电机位置。
            还要记录限位开关状态，辅助确认是否撞击。
            */
            if(DR16_Snap.S1==SW_UP){
                LOG_ERROR("System Disabled by User!");
                LOG_ERROR("Yaw Motor Angle: %.2f,Deliver L Motor Angle: %.2f,Deliver R Motor Angle: %.2f,Igniter Motor Angle: %.2f", 
                    Yawer.YawMotor.getMotorAngle(), 
                    Launcher.DeliverMotor[0].getMotorAngle(), 
                    Launcher.DeliverMotor[1].getMotorAngle(),
                     Launcher.IgniterMotor.getMotorAngle());
                LOG_ERROR("Limit Switch States: Yaw_L:%d,Yaw_R:%d,Deliver_L:%d,Deliver_R:%d,Igniter:%d", 
                    SW_YAW_L_OFF,SW_YAW_R_OFF,SW_DELIVER_L_OFF,SW_DELIVER_R_OFF,SW_IGNITER_OFF);
            }
            else if(DR16_Snap.S1==SW_MID){
                LOG_WARN("Autofire Paused by User!");
                LOG_WARN("Yaw Motor Angle: %.2f,Deliver L Motor Angle: %.2f,Deliver R Motor Angle: %.2f,Igniter Motor Angle: %.2f", 
                    Yawer.YawMotor.getMotorAngle(), 
                    Launcher.DeliverMotor[0].getMotorAngle(), 
                    Launcher.DeliverMotor[1].getMotorAngle(),
                     Launcher.IgniterMotor.getMotorAngle());
                LOG_WARN("Limit Switch States: Yaw_L:%d,Yaw_R:%d,Deliver_L:%d,Deliver_R:%d,Igniter:%d", 
                    SW_YAW_L_OFF,SW_YAW_R_OFF,SW_DELIVER_L_OFF,SW_DELIVER_R_OFF,SW_IGNITER_OFF);
            }

        }
        if (last_S2 != DR16_Snap.S2) {
            LOG_INFO("DR16 Switch Updated: S2=%s -> %s", 
                Get_SW_Str(last_S2), Get_SW_Str(DR16_Snap.S2));
            last_S2 = DR16_Snap.S2;
        }
		#endif

        // 处理遥控器连接状态及模式切换
        if (DR16_Snap.Status != DR16_ESTABLISHED) {
            Robot.Flag.Status.rc_connected = false;
            Robot.Status.current_state = SYS_DISCONNECTED;
        }
        else{
            Robot.Flag.Status.rc_connected = true;
            // 处理遥控器开关逻辑
            // 手动失能开关 S1向上
            if(DR16_Snap.S1==SW_UP){
                Robot.Cmd.sys_enable=false;
            }
            else{
                Robot.Cmd.sys_enable=true;
            }
            //自动发射模式,跳过自检 S1向下
            if(DR16_Snap.S1==SW_DOWN){
                Robot.Cmd.skip_check=true;
                Robot.Cmd.autofire_enable=true;
            }
            //s1中则是自动发射关闭,会卡在待机状态,并且不会跳过自检
            if(DR16_Snap.S1==SW_MID){
                Robot.Cmd.autofire_enable=false;
                Robot.Cmd.skip_check=false;
                Robot.Flag.Status.stop_continus_fire=false;
            }
            //s2
            //为了防止抢占校准状态,增加校准状态判断
            if(Robot.Status.current_state!=SYS_HOMING&&Yawer.is_Yaw_Init()){
                if(DR16_Snap.S2==SW_UP){
                    
                }
                else if(DR16_Snap.S2==SW_MID){
                    
                }
                else{
                    Robot.Status.yaw_control_state=YAW_MANUAL_AIM;
                }
            }
    
        }
        
        switch (Robot.Status.current_state)
        {
        case SYS_DISCONNECTED:
        {
            // 恢复条件：遥控器重连
            if (Robot.Flag.Status.rc_connected) {
                Robot.Status.current_state = SYS_MANUAL_TEST_KEY;
            }
        }
        break;
        case SYS_MANUAL_TEST_KEY:
        {
            //按键自检逻辑
            Launcher.key_check();
            //处理跳过逻辑
            if (Robot.Cmd.skip_check) {
                Launcher.check_progress = MASK_ALL_PASSED; // 强制全满
            }

            //更新全局标志位
            Robot.Flag.Check.limit_sw_ok = (Launcher.check_progress == MASK_ALL_PASSED);

            //5个按键手动检查全部通过则进入校准状态,后续可以加入电机检查
            if (Robot.Flag.Check.limit_sw_ok) {
                //自检完后,如果没有任何一个限位开关被按下时,才等于1。
                bool key_released_temp=!(SW_YAW_L_OFF||SW_YAW_R_OFF||SW_DELIVER_L_OFF||SW_DELIVER_R_OFF||SW_IGNITER_OFF);
                if(key_released_temp&&Robot.Cmd.sys_enable){
                    //上面的条件加入了系统使能判断,一是防止手动按键后突然就开始校准伤人，二是防止限位开关延时导致校准失准。
                    LOG_INFO("Manual Test Key Passed, Starting Homing Sequence.");
                    Launcher.servo_pwm_test_lock_up();
                    Launcher.start_calibration();//注意,这里启动了校准过程,会配置电机为速度环,直到撞到限位开关
                    Robot.Status.current_state=SYS_HOMING;
                }
            }

            // 日志记录手动检查进度
            static uint8_t last_check_progress = 0;
            if (last_check_progress != Launcher.check_progress) {    
                // 打印进度变化，显示后5位二进制状态
                LOG_INFO("Check Progress Updated: [" BIN5_FMT "] -> [" BIN5_FMT "]", 
                        BIN5_VAL(last_check_progress), BIN5_VAL(Launcher.check_progress));

                // 根据掩码位判断当前是哪个开关被触发（利用刚才更新的那一位）
                uint8_t triggered_bit = Launcher.check_progress ^ last_check_progress;
                if (triggered_bit & MASK_DELIVER_L) {
                    LOG_INFO("Switch Deliver L Checked! Current: [" BIN5_FMT "]", BIN5_VAL(Launcher.check_progress));   
                }
                if (triggered_bit & MASK_DELIVER_R) {
                    LOG_INFO("Switch Deliver R Checked! Current: [" BIN5_FMT "]", BIN5_VAL(Launcher.check_progress));
                }
                if (triggered_bit & MASK_IGNITER) {
                    LOG_INFO("Switch Igniter Checked! Current: [" BIN5_FMT "]", BIN5_VAL(Launcher.check_progress));
                }
                if (triggered_bit & MASK_YAW_L) {
                    LOG_INFO("Switch Yaw L Checked! Current: [" BIN5_FMT "]", BIN5_VAL(Launcher.check_progress));
                }
                if (triggered_bit & MASK_YAW_R) {
                    LOG_INFO("Switch Yaw R Checked! Current: [" BIN5_FMT "]", BIN5_VAL(Launcher.check_progress));
                }  

                last_check_progress = Launcher.check_progress;
            }

        }
        break;
        
        case SYS_HOMING:
            Robot.Status.yaw_control_state = YAW_CALIBRATING;
            //更新校准标志位
            if(Yawer.is_Yaw_L_calibrated()){
                Robot.Flag.Status.is_calibrated|=MASK_YAW_L_CALIBRATED;
            }
            if(Yawer.is_Yaw_R_calibrated()){
                Robot.Flag.Status.is_calibrated|=MASK_YAW_R_CALIBRATED;
            }
            if(Launcher.is_deliver_R_calibrated()){
                Robot.Flag.Status.is_calibrated|=MASK_DELIVER_R_CALIBRATED;
            }
            if(Launcher.is_deliver_L_calibrated()){
                Robot.Flag.Status.is_calibrated|=MASK_DELIVER_L_CALIBRATED;
            }
            if(Launcher.is_igniter_calibrated()){
                Launcher.target_igniter_angle=IGNITER_OFFSET_POS;  // 回到缓冲位置
                Robot.Flag.Status.is_calibrated|=MASK_IGNITER_CALIBRATED;
            }
            //校准完毕后角度环到安全位置，到达后停止
            if(Yawer.is_Yaw_Init()){
                Robot.Status.yaw_control_state = YAW_MANUAL_AIM; //校准完成后，进入手动模式
                Yawer.mode_YAW = MODE_ANGLE; //切换回角度环
                Yawer.yaw_target=0;//回到中间位置
            }
            if(Launcher.is_calibrated()){
                Launcher.target_igniter_angle=IGNITER_OFFSET_POS;  // 回到缓冲位置
                /*
                Launcher.target_deliver_angle=POS_BUFFER;   // 回缓冲位置
                由于发射流程的校准需求,在check_calibration_logic();配置了角度环设置和回缓冲位置,这里就不设置了。
                */
            }

            //日志记录校准位变化
            static uint8_t last_calibration_progress = 0;
            if (last_calibration_progress != Robot.Flag.Status.is_calibrated) {
                LOG_INFO("Calibration Progress Updated: 0x%02X -> 0x%02X", last_calibration_progress, Robot.Flag.Status.is_calibrated);
                last_calibration_progress = Robot.Flag.Status.is_calibrated;
            }

            //检查限位开关并处理校准逻辑
            Launcher.check_calibration_logic();
            
            //全部校准完毕后，切换到待机状态
            if (Robot.Flag.Status.is_calibrated==MASK_ALL_CALIBRATED) {
                Robot.Status.current_state = SYS_READY;
            }
            break;
        //校准完毕时，总有一个限位开关被触发，防止误触发限位开关进入error状态
        //因此加一个校准完毕后给角度环到安全位置的状态停留，直到都到达指定位置后再进入待机状态
        case SYS_READY:
        {
            if(Launcher.is_deliver_at_target(5)&&Launcher.is_igniter_at_target(5)&&Yawer.isMotorAngleReached(5.0f))
            {
                Robot.Status.current_state = SYS_AUTOFIRE_SUSPEND;
                LOG_INFO("System Reseted after calibration");
            }

        }
            break;

        case SYS_AUTOFIRE_SUSPEND:
            // --- 待机（发射暂停）状态 ---
            // 发射指令检测
            if (Robot.Cmd.autofire_enable&& !Robot.Flag.Status.stop_continus_fire) {
                Robot.Status.current_state = SYS_AUTO_PREP;
                LOG_INFO("Auto Fire Enabled");
            }
            if(Launcher.mode_deliver[0]==MODE_SPEED||Launcher.mode_deliver[1]==MODE_SPEED){
                #if 0
                //由于自动发射中增加了滑块电机速度环校准逻辑，因此在待机中如果是速度环模式一定要失能滑块电机。（不然不会检查停止条件，会直接失控撞限位开关）
                Launcher.pid_deliver_spd[0].Target=0;
                Launcher.pid_deliver_spd[1].Target=0;
                #else
                //但是也可以继续校准，只要在这里检测就行。校准完后会回缓冲
                Launcher.check_calibration_logic();
                #endif
            }
			//else
                //Launcher.target_deliver_angle=(POS_BUFFER);
            break;
            
        case SYS_AUTO_PREP:
            // --- 自动发射准备 ---
            #if 0
            Launcher.target_igniter_angle=POS_IGNITER;  // 默认发射力度
            #else
            Launcher.target_igniter_angle=Launcher.IgniterMotor.getMotorTotalAngle();
			#endif
            // 检查是否到位
            if (Launcher.is_igniter_at_target(5)) 
            {  
                Robot.Status.current_state = SYS_AUTO_FIRE;
                LOG_INFO("default Igniter ready");
            }
            break;
            
        case SYS_AUTO_FIRE:
            Launcher.Run_Firing_Sequence();
            //S1不为下时，会重置到待机模式 (Run_Firing_Sequence 内部也会处理打断复位)
            if (!Robot.Cmd.autofire_enable){
                Robot.Status.current_state = SYS_AUTOFIRE_SUSPEND;
                LOG_WARN("Auto Fire Aborted");
            }
            
            break;
        }

        //yaw轴子状态机,包含状态如下
        /*
            YAW_MANUAL_AIM:手动瞄准
            YAW_VISION_AIM:视觉瞄准
            YAW_CORRECT_AIM调参板瞄准
            YAW_DISABLE_MOTOR:失能电机
            yaw_calibrating:校准模式
        */
        Yawer.yaw_state_machine(&Robot.Status.yaw_control_state, DR16_Snap.RX_Norm, DR16_Snap.RY_Norm);

		//计算 PID
        /*
        这里不放在使能里面一是积分效果不强，
        而是方便我失能电机也能看pid输出。
        因为debug时随时要失能电机看参数。（为了流程安全，失能是必要的）
        */
		Launcher.adjust();
		Yawer.adjust();
			
        if (Robot.Status.current_state != SYS_DISCONNECTED && 
            Robot.Status.current_state != SYS_MANUAL_TEST_KEY&&
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
            //舵机测试放在这里，这样只需要失能就可以测试舵机
            if (Debugger.enable_debug_mode==1) {
				Launcher.key_check();
				Robot.Flag.Check.limit_sw_ok = (Launcher.check_progress == MASK_ALL_PASSED);

            }
            else if(Debugger.enable_debug_mode==2){
                Launcher.servo_pwm_test_unlock_down();
            }
            else if(Debugger.enable_debug_mode==3){
                Launcher.servo_pwm_test_lock_up();
            }
        }
        // 记录主状态切换
        static System_State_e last_state = SYS_DISCONNECTED;
        if (Robot.Status.current_state != last_state) 
        {
            #if enum_X_Macros_disable
            LOG_INFO("System State Change: %d -> %d", last_state, Robot.Status.current_state);
            #else
            LOG_INFO("System State Change: %s -> %s", System_State_To_Str(last_state), System_State_To_Str(Robot.Status.current_state));
            #endif
            last_state = Robot.Status.current_state;
        }

        MotorMsgPack(Tx_Buff1, Yawer.YawMotor);
        xQueueSend(CAN2_TxPort, &Tx_Buff1.Id1ff, 0);
        //R=0，L=1
        MotorMsgPack(Tx_Buff, Launcher.DeliverMotor[1], Launcher.DeliverMotor[0], Launcher.IgniterMotor);
		xQueueSend(CAN1_TxPort, &Tx_Buff.Id200, 0);

    }
}


