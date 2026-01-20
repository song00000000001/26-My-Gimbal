#include "launcher_driver.h"
#include "robot_config.h"
#include "global_data.h"

  
/*debug中的参数debug_fire_type可以实时调整发射类型：
1为单发第一发，
2为单发第二发，
3为单发第三发，
0或其他为连发一二三四。
注意，默认设置在launcher_task中设置，目前是0。
*/

/*
//todo: 
song
视觉瞄准到位触发
Robot.Cmd.fire_command=true;
if (Robot.Cmd.fire_command&&is_deliver_at_target(5)) {
    fire_state = FIRE_PULL_LOAD;
}
但是往年经验是不需要等视觉。
*/

#define aim_wait_test 1

/*发射状态机*/
void Launcher_Driver::Run_Firing_Sequence()
{
    static uint32_t state_timer = 0;
    uint32_t current_time = xTaskGetTickCount();
    bool aim_reached_or_timeout = false;

    //记录debug_fire_type变化
    static uint8_t last_debug_fire_type=0;
    if(last_debug_fire_type!=Debugger.debug_fire_type)
    {
        LOG_WARN("debug_fire_type changed: %d -> %d",last_debug_fire_type,Debugger.debug_fire_type);
        last_debug_fire_type=Debugger.debug_fire_type;
        //考虑切换后重置状态机，但是可能会影响正在发射的流程，所以还是不重置了。
    }    
    switch (fire_state)
    {
        case FIRE_IDLE:
            //确保滑块在缓冲区
            //target_deliver_angle=(POS_BUFFER);
            servo_igniter_lock; // 锁止扳机舵机
			servo_transfomer_lock;// 锁止卡镖舵机
            loader_target_mode=LOAD_STOWED;//确保升降机在收起位置
            //这里后续会修改行程电机位置，故加入检查条件
            if (Launcher.is_igniter_at_target(5)) {
				state_timer = current_time;
                //根据debug_fire_type选择发射类型
                if(Debugger.debug_fire_type==2)
                {
                    state_timer = current_time;
                    fire_state = FIRE_CALIBRATION_2;
                    start_deliver_calibration();
                }
                else if(Debugger.debug_fire_type==3)
                {
                    state_timer = current_time;
                    fire_state = FIRE_RELOAD_LIFT_3;
                }
                else
                {
                    state_timer = current_time;
                    fire_state = FIRE_CALIBRATION_1;
                    start_deliver_calibration();
                }
            }
            break;

        //滑块校准
        case FIRE_CALIBRATION_1:
            check_calibration_logic();
            //如果真出现限位开关触碰时机不同,导致实际需要一点时间进行同步的话，这里的条件是有问题的，这样会导致边拉下边同步，很危险，容易损失滑台。
            //所以需要增加同步误差检测，确保同步误差在允许范围内才进入下一状态。
            if(is_calibrated()&&is_deliver_sync_ok(Debugger.deliver_sync_threshold)){
                state_timer = current_time;
                fire_state = FIRE_PULL_DOWN_1;
            }
            break;

        //下拉滑块到底部扳机
        case FIRE_PULL_DOWN_1:
            target_deliver_angle=(POS_BOTTOM);
            static  uint16_t pull_down_counter = 0;
            //防止滑块卡住无法到底部
            pull_down_counter++;
            //如果超过4秒还没到底部就强制进入下一状态
            if(pull_down_counter>4000){
                LOG_ERROR("Deliver Motor Pull Down Timeout! Forcing Next State.");
                pull_down_counter=0;
                state_timer = current_time;
                fire_state = FIRE_WAIT_BOTTOM_1;
            }
            if (is_deliver_at_target(5)) {
                state_timer = current_time;
                fire_state = FIRE_WAIT_BOTTOM_1;
                pull_down_counter=0;//重置计数器
            }
            break;

        // 底部等待
        case FIRE_WAIT_BOTTOM_1:
            if ((current_time - state_timer) > fire_sequence_delay_params.put_delay) {
                state_timer = current_time;
                fire_state = FIRE_RETURN_UP_1;
            }
            break;

        // 滑块回缓冲区
        case FIRE_RETURN_UP_1:
            target_deliver_angle=(POS_BUFFER);
            if (is_deliver_at_target(5)) {
                state_timer = current_time;
                fire_state = FIRE_WAIT_UP_1;
            }
            break;

        // 缓冲区等待
        case FIRE_WAIT_UP_1:
            if ((current_time - state_timer) > fire_sequence_delay_params.before_fire_delay) {
                state_timer = current_time;
                /*这条日志虽然直观来看应该放在射击状态里，
                但实际上是顺序执行并且跳转很快，
                几乎不会出现触发了log但是瞬间单片机暂停打断的情况。
                放在这里可以实现只触发一次的效果，避免发射状态延时期间重复打印日志。
                */
				fire_state = FIRE_WAIT_AIM_1;
                LOG_WARN("Dart1 Fired! Total Count: %d", Robot.Status.dart_count);
            }
            break;    

        // 等待瞄准完成，增加超时逻辑，非必要等待
        case FIRE_WAIT_AIM_1:  
            /*todo
                song
                这里往年是直接发射，即相信调参板/视觉在发射前的3s多内已经微调好了位置。
                现在增加到位检测，并增加超时就继续发射逻辑，以免卡住。
                后续可以考虑增加一个视觉瞄准完成的标志位，然后在这里等待该标志位。
                这样可以兼顾安全和瞄准。
            */  
            #if aim_wait_test
            aim_reached_or_timeout = (is_igniter_at_target(5)&&Yawer.isMotorAngleReached(5.0f))
            ||((current_time - state_timer)>fire_sequence_delay_params.wait_for_aim_delay);
            if (aim_reached_or_timeout) 
            {
                state_timer = current_time;
                fire_state = FIRE_SHOOTING_1;
                aim_reached_or_timeout = false;
                if(((current_time - state_timer)>fire_sequence_delay_params.wait_for_aim_delay))
                {
                    LOG_ERROR("Dart1 Aim Wait Timeout! Forcing Fire.");
                }
                LOG_WARN("Dart1 Fired! Total Count: %d", Robot.Status.dart_count);
            }
            #else
            state_timer = current_time;
            fire_state = FIRE_SHOOTING_1;
            LOG_WARN("Dart1 Fired! Total Count: %d", Robot.Status.dart_count + 1);
            #endif

            break;
        
        // 射击
        case FIRE_SHOOTING_1:
            servo_igniter_unlock; // 解锁扳机舵机，发射
           
            // 等待发射完成
            if ((current_time - state_timer) > fire_sequence_delay_params.after_fire_delay) {
                state_timer= current_time;
                Robot.Status.dart_count++; // 计数+1
                /*上面的发射计数可以作为触发器,在yaw_control_task中检测变化或者参数，
                然后控制系统根据调参板数据开始微调yaw和行程电机位置。
                然后在下一发发射前检查位置是否到位。并增加超时逻辑。
                */
                servo_igniter_lock;
                if(Debugger.debug_fire_type==1)
                {
                    fire_state = FIRE_IDLE;
                    Robot.Flag.Status.stop_continus_fire=true;
                    Robot.Status.current_state = SYS_AUTOFIRE_SUSPEND;
                }
                else
                {
                    fire_state = FIRE_CALIBRATION_2;
                    start_deliver_calibration();
                }
            }
            break;

        //滑块校准
        case FIRE_CALIBRATION_2:
            check_calibration_logic();
            if(is_calibrated()&&is_deliver_sync_ok(Debugger.deliver_sync_threshold)){
                state_timer = current_time;
                fire_state = FIRE_PULL_DOWN_2;
            }
            break;

        // 滑块回底部
        case FIRE_PULL_DOWN_2:
            loader_target_mode=LOAD_DYNAMIC_SYNC;
            target_deliver_angle=(POS_BOTTOM);
            if (is_deliver_at_target(5)) {
                state_timer = current_time;
                fire_state = FIRE_WAIT_BOTTOM_2;
                loader_target_mode=LOAD_PRE_LOAD;
            }  
            break;
            
        //底部等待
        case FIRE_WAIT_BOTTOM_2:
            if ((current_time - state_timer) > fire_sequence_delay_params.put_delay) {
                state_timer = current_time;
                fire_state = FIRE_RETURN_UP_2;
            }
            break;

        // 滑块回缓冲区
        case FIRE_RETURN_UP_2:
            target_deliver_angle=(POS_BUFFER);
            if (is_deliver_at_target(5)) {
                state_timer = current_time;
                fire_state = FIRE_WAIT_UP_2;
                loader_target_mode=LOAD_ENGAGED;
            }
            break;
        // 缓冲区等待(必要的等待时间，确保滑块稳定在缓冲区，确保升降机和镖体分离并稳定)
        case FIRE_WAIT_UP_2:
            if ((current_time - state_timer) > fire_sequence_delay_params.before_fire_delay) {
                state_timer = current_time;
                fire_state = FIRE_SHOOTING_2;
            }
            break;

        // 等待瞄准完成，增加超时逻辑，非必要等待
        case FIRE_WAIT_AIM_2:
            #if aim_wait_test
            aim_reached_or_timeout = (is_igniter_at_target(5)&&Yawer.isMotorAngleReached(5.0f))
            ||((current_time - state_timer)>fire_sequence_delay_params.wait_for_aim_delay);
            if (aim_reached_or_timeout) 
            {
                state_timer = current_time;
                fire_state = FIRE_SHOOTING_2;
                aim_reached_or_timeout = false;
                if(((current_time - state_timer)>fire_sequence_delay_params.wait_for_aim_delay))
                {
                    LOG_ERROR("Dart2 Aim Wait Timeout! Forcing Fire.");
                }
                LOG_WARN("Dart2 Fired! Total Count: %d", Robot.Status.dart_count);
            }
            #else
            state_timer = current_time;
            fire_state = FIRE_SHOOTING_2;
            LOG_WARN("Dart2 Fired! Total Count: %d", Robot.Status.dart_count + 1);
            #endif

            break;

        // 射击    
        case FIRE_SHOOTING_2:
            servo_igniter_unlock; // 解锁扳机舵机，发射
            if ((current_time - state_timer) > fire_sequence_delay_params.after_fire_delay) {
                Robot.Status.dart_count++; // 计数+1
                servo_igniter_lock;
                state_timer= current_time;
                fire_state = FIRE_RELOAD_LIFT_3;
            }
            break;

        // 升降机上升，准备装填第三发
        case FIRE_RELOAD_LIFT_3:
            loader_target_mode=LOAD_STOWED;
            if ((current_time - state_timer) > fire_sequence_delay_params.loader_up_delay) {
                state_timer = current_time;
                if(Debugger.debug_fire_type==2)
                {
                    fire_state = FIRE_IDLE;
                    Robot.Flag.Status.stop_continus_fire=true;
                    Robot.Status.current_state = SYS_AUTOFIRE_SUSPEND;
                }
                else
                {
                    fire_state = FIRE_RELOAD_RELEASE_3;
                }
            }
            break;

        // 卡镖释放，转移下一发到发射区
        case FIRE_RELOAD_RELEASE_3:
            servo_transfomer_unlock; // 松开卡镖舵机，转移下一发到发射区
            if ((current_time - state_timer) > fire_sequence_delay_params.relapse_delay) {
                servo_transfomer_lock; // 重新卡住卡镖舵机
                state_timer = current_time;
                fire_state = FIRE_CALIBRATION_3;
                start_deliver_calibration();
            }
            break;
        
        // 校准滑块
        case FIRE_CALIBRATION_3:
            check_calibration_logic();
            if(is_calibrated()&&is_deliver_sync_ok(Debugger.deliver_sync_threshold)) {
                state_timer = current_time;
                fire_state = FIRE_PULL_DOWN_3;
            }
            break;

        //下拉滑块到底部扳机
        case FIRE_PULL_DOWN_3:
            loader_target_mode=LOAD_DYNAMIC_SYNC;
            target_deliver_angle=(POS_BOTTOM);
            if (is_deliver_at_target(5)) {
                state_timer = current_time;
                fire_state = FIRE_WAIT_BOTTOM_3;
                loader_target_mode=LOAD_PRE_LOAD;
            }
            break;    

        // 底部等待
        case FIRE_WAIT_BOTTOM_3:
            if ((current_time - state_timer) > fire_sequence_delay_params.put_delay) {
                state_timer = current_time;
                fire_state = FIRE_RETURN_UP_3;
            }
            break;

        // 滑块回缓冲区
        case FIRE_RETURN_UP_3:
            target_deliver_angle=(POS_BUFFER);
            if (is_deliver_at_target(5)) {
                state_timer = current_time;
                fire_state = FIRE_WAIT_UP_3;
                loader_target_mode=LOAD_ENGAGED;
            }
            break;

        // 缓冲区等待
        // 升降机下降，装填下一发
        case FIRE_WAIT_UP_3:
            if ((current_time - state_timer) > fire_sequence_delay_params.before_fire_delay) {
                state_timer = current_time;
                fire_state = FIRE_WAIT_AIM_3;
            }
            break;

        // 等待瞄准完成，增加超时逻辑，非必要等待
        case FIRE_WAIT_AIM_3:
            #if aim_wait_test
            aim_reached_or_timeout = (is_igniter_at_target(5)&&Yawer.isMotorAngleReached(5.0f))
            ||((current_time - state_timer)>fire_sequence_delay_params.wait_for_aim_delay);
            if (aim_reached_or_timeout) {
                state_timer = current_time;
                fire_state = FIRE_SHOOTING_3;
                aim_reached_or_timeout = false;
                if(((current_time - state_timer)>fire_sequence_delay_params.wait_for_aim_delay))
                {
                    LOG_ERROR("Dart3 Aim Wait Timeout! Forcing Fire.");
                }
                LOG_WARN("Dart3 Fired! Total Count: %d", Robot.Status.dart_count);
            }
            #else
            state_timer = current_time;
            fire_state = FIRE_SHOOTING_3;
            LOG_WARN("Dart3 Fired! Total Count: %d", Robot.Status.dart_count + 1);
            #endif
            break;
        
        // 射击
        case FIRE_SHOOTING_3:
            servo_igniter_unlock; // 解锁扳机舵机，发射
            if ((current_time - state_timer) > fire_sequence_delay_params.after_fire_delay) {
                Robot.Status.dart_count++; // 计数+1
                servo_igniter_lock;
                state_timer= current_time;
                if(Debugger.debug_fire_type==3)
                {
                    fire_state = FIRE_IDLE;
                    Robot.Flag.Status.stop_continus_fire=true;
                    Robot.Status.current_state = SYS_AUTOFIRE_SUSPEND;
                }
                else
                {
                    fire_state = FIRE_RELOAD_LIFT_4;
                }
            }
            break;

        // 升降机上升，准备装填第四发
        case FIRE_RELOAD_LIFT_4:
            loader_target_mode=LOAD_STOWED;
            if ((current_time - state_timer) >  fire_sequence_delay_params.loader_up_delay) {
                state_timer = current_time;
                fire_state = FIRE_RELOAD_RELEASE_4;
            }
            break;

        // 卡镖释放，转移下一发到发射区
        case FIRE_RELOAD_RELEASE_4:
            servo_transfomer_unlock; // 松开卡镖舵机，转移下一发到发射区
            if ((current_time - state_timer) > fire_sequence_delay_params.relapse_delay) {
                servo_transfomer_lock; // 重新卡住卡镖舵机
                state_timer = current_time;
                fire_state = FIRE_CALIBRATION_4;
                start_deliver_calibration();
            }
            break;

        // 校准滑块
        case FIRE_CALIBRATION_4:
            check_calibration_logic();
            if(is_calibrated()&&is_deliver_sync_ok(Debugger.deliver_sync_threshold)){
                state_timer = current_time;
                fire_state = FIRE_PULL_DOWN_4;
            }
            break;

        //下拉滑块到底部扳机
        case FIRE_PULL_DOWN_4:
            loader_target_mode=LOAD_DYNAMIC_SYNC;
            target_deliver_angle=(POS_BOTTOM);
            if (is_deliver_at_target(5)) {
                state_timer = current_time;
                fire_state = FIRE_WAIT_BOTTOM_4;
                loader_target_mode=LOAD_PRE_LOAD;
            }
            break;
        
        // 底部等待
        case FIRE_WAIT_BOTTOM_4:
            if ((current_time - state_timer) > fire_sequence_delay_params.put_delay) {
                state_timer = current_time;
                fire_state = FIRE_RETURN_UP_4;
            }
            break;
        
        // 滑块回缓冲区
        case FIRE_RETURN_UP_4:
            target_deliver_angle=(POS_BUFFER);
            if (is_deliver_at_target(5)) {
                state_timer = current_time;
                fire_state = FIRE_WAIT_UP_4;
                loader_target_mode=LOAD_ENGAGED; // 升降机下降，装填第四发
            }
            break;

        // 缓冲区等待
        case FIRE_WAIT_UP_4:
            if ((current_time - state_timer) > fire_sequence_delay_params.before_fire_delay) {
                state_timer = current_time;
                fire_state = FIRE_WAIT_AIM_4; // 完成发射，回到闲置状态
            }
            break;

        // 等待瞄准完成，增加超时逻辑，非必要等待      
        case FIRE_WAIT_AIM_4:
            #if aim_wait_test
            aim_reached_or_timeout = (is_igniter_at_target(5)&&Yawer.isMotorAngleReached(5.0f))
            ||((current_time - state_timer)>fire_sequence_delay_params.wait_for_aim_delay);
            if (aim_reached_or_timeout) 
            {
                state_timer = current_time;
                fire_state = FIRE_SHOOTING_4;
                aim_reached_or_timeout = false;
                if(((current_time - state_timer)>fire_sequence_delay_params.wait_for_aim_delay))
                {
                    LOG_ERROR("Dart4 Aim Wait Timeout! Forcing Fire.");
                }
                LOG_WARN("Dart4 Fired! Total Count: %d", Robot.Status.dart_count);
            }
            #else
            state_timer = current_time;
            fire_state = FIRE_SHOOTING_4;
            LOG_WARN("Dart4 Fired! Total Count: %d", Robot.Status.dart_count + 1);
            #endif
            break;

        // 射击    
        case FIRE_SHOOTING_4:
            servo_igniter_unlock; // 解锁扳机舵机，发射
            if ((current_time - state_timer) > fire_sequence_delay_params.after_fire_delay) {
                Robot.Status.dart_count++; // 计数+1
                servo_igniter_lock;
                state_timer= current_time;
                fire_state = FIRE_IDLE; // 完成发射，回到闲置状态
                //每打4发,就需要将S1回中再下按,否则不会继续发射
                //起始值为1,发射后变为2,3,4,然后5%4又变为1
                if(Robot.Status.dart_count%4==1){
                    Robot.Flag.Status.stop_continus_fire=true;
                    Robot.Status.current_state = SYS_AUTOFIRE_SUSPEND;
                }
            }
            break;    

        default:
            fire_state = FIRE_IDLE;
            break;
    }
    // 记录发射子状态切换
    static Fire_State_e last_fire_state = FIRE_IDLE;
    if (fire_state != last_fire_state) 
    {
        #if enum_X_Macros_disable
        LOG_WARN("Fire State Change: %d -> %d", last_fire_state,fire_state);
        #else
        LOG_INFO("Fire State Change: %s -> %s", Fire_State_To_Str(last_fire_state), Fire_State_To_Str(fire_state));
        #endif
        last_fire_state = fire_state;
    }
}

