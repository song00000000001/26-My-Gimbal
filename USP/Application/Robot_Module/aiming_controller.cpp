
#include "internal.h"
#include "global_data.h"
#include "robot_config.h"

/**
 * @brief Yaw轴控制任务
 * @parma None
 * @return None
 */



//尝试合并装填任务,不知道能不能直接在can发送队列里连续发两次
static myPID load_pid_ang;									// 装填电机的编码器环
static myPID load_pid_spd;									// 装填电机的速度环


void key_check(){  
    // 初始状态设为自检
    Robot.Status.current_state = SYS_CHECKING;
    // 记录哪几个开关已经检测过了 (Bitmask)
    static uint8_t check_progress = 0; 
    if(SW_YAW_L_OFF){
        check_progress |= MASK_YAW_L;
    }
    if(SW_YAW_R_OFF){
        check_progress |= MASK_YAW_R;
    }
    if(SW_DELIVER_L_OFF){
        check_progress |= MASK_DELIVER_L;
    }
    if(SW_DELIVER_R_OFF){
        check_progress |= MASK_DELIVER_R;
    }
    if(SW_IGNITER_OFF){
        check_progress |= MASK_IGNITER;
    }
    // 3. 处理跳过逻辑
    if (Robot.Cmd.skip_check) {
        check_progress = MASK_ALL_PASSED; // 强制全满
    }

    // 4. 更新全局标志位
    Robot.Flag.Check.limit_sw_ok = (check_progress == MASK_ALL_PASSED);
   
}

void stop_all_motor(){
    Launcher.stop(); 
    Yawer.disable();
    load_pid_ang.clean_intergral();
    load_pid_spd.clean_intergral();
    Launcher.fire_lock();
}

//yaw摇杆控制函数
void yaw_remote_control_task(){

    if(DR16.GetS2() == SW_UP) // 右拨杆上,取消使能
    {
        Robot.Status.yaw_control_state = disable_motor;
    }
    else if(Robot.Status.yaw_control_state!=YAW_CALIBRATING)
    {
        Robot.Status.yaw_control_state=MANUAL_AIM;
    }
}

void yaw_state_machine(){
    
    static float yaw_target = 0;//, yaw_goal = 0, igniter_target_pos = 0, igniter_goal_pos = 0;
    static float yaw_correct_angle;        //yaw轴修正角
    static float default_yaw_target[2]; // 默认前哨站和基地角度

    switch (Robot.Status.yaw_control_state)
    {
    case MANUAL_AIM:
        yaw_target -= DR16.Get_LX_Norm() * 0.002f;
        yaw_target = std_lib::constrain(yaw_target, -10.2f, 10.2f);
        Yawer.update(yaw_target);
        break;
    case CORRECT_AIM:
        //根据目标选择修正角度
        //固定修正值模式
        Yawer.update(yaw_correct_angle + default_yaw_target[HitTarget]); // 更改Yaw轴角度
        break;
    case VISION_AIM:
        //视觉模式
        //todo
        {/*todo
        测试时，暂时不管视觉
        storage_base_angle = default_yaw_target[HitTarget];

        if (vision_recv_pack.ros == 1)
        {
            yaw_target += 0.0003;
        }
        if (vision_recv_pack.ros == 2)
        {
            yaw_target -= 0.0003;
        }
        if (vision_recv_pack.ros == 0)
        {
            yaw_target += 0;
        }
        yaw_target = std_lib::constrain(yaw_target, -10.2f, 10.2f);
        Yawer.update(yaw_target);
        default_yaw_target[HitTarget] = yaw_target;
        //计算电机pid
        Yawer.adjust();
        */
       }
        break;
    case YAW_CALIBRATING:
        //校准模式
        //进行校准，校准完成后，自动改变校准标志
        Yawer.calibration();
        if(Yawer.is_Yaw_Init()){
            Robot.Status.yaw_control_state = MANUAL_AIM; //校准完成后，进入手动模式
        }
        break;
    case disable_motor:
    default:
        Yawer.disable();
        break;
    }

}


//注意校准模式，下面的状态机本来是发射的，但是临时拿来测试，后续要删掉，还给发射任务
//而后续yaw校准会靠发射任务将robot.status.yaw_control_state设为校准状态
//yaw任务的子状态机再根据状态进行校准

void yaw_and_load_motor_control(){
    //这里为了方便,脱离状态机单独处理可能的输出。
    if(Robot.Status.yaw_control_state!=disable_motor&& Robot.Status.current_state != SYS_OFFLINE&&Robot.Status.current_state!=SYS_CHECKING)
    {
        //计算电机pid
        Yawer.adjust();
    }
    else
    {
        Yawer.disable();
    }

    if(Robot.Status.current_state != SYS_OFFLINE&&Robot.Status.current_state!=SYS_CHECKING)
    {
        //装填任务
        //计算装填电机pid
        load_pid_ang.Current = loadermotor.getMotorTotalAngle();//换成getTotalAngle试试,测试后发现,total才是绝对,直接get的会有跳变
        load_pid_ang.Adjust();

        load_pid_spd.Target = load_pid_ang.Out;
        load_pid_spd.Current = loadermotor.getMotorSpeed();
        load_pid_spd.Adjust();

        //设置电机输出
        loadermotor.setMotorCurrentOut(load_pid_spd.Out);
    }
    else
    {
        load_pid_ang.clean_intergral();
        load_pid_spd.clean_intergral();
        loadermotor.setMotorCurrentOut(0);
    }
}
		   
uint16_t servo_ccr_test =200;

void Yaw_Task(void *arg)
{
    Motor_CAN_COB Tx_Buff;
	Motor_CAN_COB Tx_Buff1;
	TickType_t xLastWakeTime_t;

    //初始化参数可以考虑集成到launcher中
    load_pid_ang.SetPIDParam(10, 0, 0, 100, 1200);
	load_pid_spd.SetPIDParam(50, 1, 0, 1000, 16000);
    load_pid_ang.Target = 60;    //安全位置

	// 初始状态设为自检
    //测试用,只执行一次
    // 测试舵机动作
    //test_servo_action(); 
	 
    // 任务频率控制
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1);
	
	for (;;)
	{
		vTaskDelayUntil(&xLastWakeTime_t, xFrequency);
        
        //s2摇杆解析
        yaw_remote_control_task();
        //独立状态机运行，但受到状态管控，主要是并行任务
        //根据s2摇杆切换失能，手动，修正，视觉（暂无）模式
        yaw_state_machine();
        //yaw轴和装填电机控制，都有失能保护，yaw轴还有初始化保护
        yaw_and_load_motor_control();

	    /*打包发送*/
        MotorMsgPack(Tx_Buff1, Yawer.YawMotor);
        xQueueSend(CAN2_TxPort, &Tx_Buff1.Id1ff, 0);
     
        //不知道能不能这样写?
        //装填电机gm6020-id2，用此命令 
        MotorMsgPack(Tx_Buff, loadermotor);
        xQueueSend(CAN2_TxPort, &Tx_Buff.Id2ff, 0); 
       
	}
}