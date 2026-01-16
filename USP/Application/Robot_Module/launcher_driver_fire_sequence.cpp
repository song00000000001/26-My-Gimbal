#include "launcher_driver.h"
#include "robot_config.h"
#include "global_data.h"

uint16_t before_fire_delay=300,put_delay=300,after_fire_delay=1000,relapse_delay=100,loader_up_delay=200;

  
/*发射状态机*/
void Launcher_Driver::Run_Firing_Sequence()
{
    static uint32_t state_timer = 0;
    uint32_t current_time = xTaskGetTickCount();
    switch (fire_state)
    {
        case FIRE_IDLE:
            //确保滑块在缓冲区
            //target_deliver_angle=(POS_BUFFER);
            servo_igniter_lock; // 锁止扳机舵机
			servo_transfomer_lock;
            loader_target_mode=LOAD_MODE_UP;
            if (1||is_deliver_at_target(5)) {
				state_timer = current_time;
                fire_state = FIRE_IGNITER_DELAY;
            }
            break;
        //点火延时
		case FIRE_IGNITER_DELAY:
			if ((current_time - state_timer) > before_fire_delay) {
                state_timer = current_time;
                fire_state = FIRE_CALIBRATION_1;
                start_deliver_calibration();
            }
			break;

        //滑块校准
        case FIRE_CALIBRATION_1:
            check_calibration_logic();
            if(is_calibrated()){
                state_timer = current_time;
                fire_state = FIRE_PULL_DOWN_1;
            }
            break;

        //下拉滑块到底部扳机
        case FIRE_PULL_DOWN_1:
            target_deliver_angle=(POS_BOTTOM);
            if (is_deliver_at_target(5)) {
                state_timer = current_time;
                fire_state = FIRE_WAIT_BOTTOM_1;
            }
            break;

        // 底部等待
        case FIRE_WAIT_BOTTOM_1:
            if ((current_time - state_timer) > 1) {
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
            if ((current_time - state_timer) > before_fire_delay) {
                state_timer = current_time;
                
                //测试时为了方便实现拨一次杆打一发,在这里修改跳转逻辑
                //恢复多发逻辑。
                #if 0
                    fire_state = FIRE_SHOOTING_4;
                #else
                    fire_state = FIRE_SHOOTING_1;
                #endif
            }
            break;    

        // 射击
        case FIRE_SHOOTING_1:
            servo_igniter_unlock; // 解锁扳机舵机，发射
            LOG_INFO("Dart Fired! Total Count: %d", Robot.Status.dart_count + 1);
            // 等待发射完成
            if ((current_time - state_timer) > after_fire_delay) {
                state_timer= current_time;
                Robot.Status.dart_count++; // 计数+1
                servo_igniter_lock;
                fire_state = FIRE_CALIBRATION_2;
                start_deliver_calibration();
            }
            break;

        //滑块校准
        case FIRE_CALIBRATION_2:
            check_calibration_logic();
            if(is_calibrated()){
                state_timer = current_time;
                fire_state = FIRE_PULL_DOWN_2;
            }
            break;

        // 滑块回底部
        case FIRE_PULL_DOWN_2:
            loader_target_mode=LOAD_MODE_FOLLOW;
            target_deliver_angle=(POS_BOTTOM);
            /*todo
            song
            这里往年是直接发射，即相信视觉在发射前已经调整好了位置。
            目前不加yaw和行程条件的原因是，如果滑块拉到底了，却一直不发射，很危险。除非加更复杂的超时逻辑。所以先这样，一步步来。
            后面的发射流程基本也要加，因为是一镖一参。
            */
            #if 0
            //这里在发射前yaw和行程电机可能会根据调参板或者视觉微调，故加入所有电机检查条件
            if (is_deliver_at_target(5)&&is_igniter_at_target(5)&&Yawer.isMotorAngleReached(5.0f)) {
                state_timer = current_time;
                fire_state = FIRE_WAIT_BOTTOM_2;
            }
            #else
            if (is_deliver_at_target(5)) {
                state_timer = current_time;
                fire_state = FIRE_WAIT_BOTTOM_2;
                loader_target_mode=LOAD_MODE_PARAL;
            }
            #endif
                
            break;
            
        //底部等待
        case FIRE_WAIT_BOTTOM_2:
            if ((current_time - state_timer) > put_delay) {
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
                loader_target_mode=LOAD_MODE_FULL_DOWN;
            }
            break;
        // 缓冲区等待
        case FIRE_WAIT_UP_2:
            if ((current_time - state_timer) > before_fire_delay) {
                state_timer = current_time;
                fire_state = FIRE_RELOAD_LOWER_2;
            }
            break;
        /*todo
        song
        这里可以和上面的缓冲区等待合并
        */
        // 升降机下降，装填下一发
        case FIRE_RELOAD_LOWER_2:
            if ((current_time - state_timer) >1) {
                state_timer = current_time;
                fire_state = FIRE_SHOOTING_2;
            }
            break;

        // 射击    
        case FIRE_SHOOTING_2:
            servo_igniter_unlock; // 解锁扳机舵机，发射
            LOG_INFO("Dart Fired! Total Count: %d", Robot.Status.dart_count + 1);
            if ((current_time - state_timer) > after_fire_delay) {
                Robot.Status.dart_count++; // 计数+1
                servo_igniter_lock;
                state_timer= current_time;
                fire_state = FIRE_RELOAD_LIFT_3;
            }
            break;

        // 升降机上升，准备装填第三发
        case FIRE_RELOAD_LIFT_3:
            loader_target_mode=LOAD_MODE_UP;
            if ((current_time - state_timer) >loader_up_delay) {
                state_timer = current_time;
                fire_state = FIRE_RELOAD_RELEASE_3;
            }
            break;

        // 卡镖释放，转移下一发到发射区
        case FIRE_RELOAD_RELEASE_3:
            servo_transfomer_unlock; // 松开卡镖舵机，转移下一发到发射区
            if ((current_time - state_timer) >relapse_delay) {
                servo_transfomer_lock; // 重新卡住卡镖舵机
                state_timer = current_time;
                fire_state = FIRE_CALIBRATION_3;
                start_deliver_calibration();
            }
            break;
        
        // 校准滑块
        case FIRE_CALIBRATION_3:
            check_calibration_logic();
            if(is_calibrated()){
                state_timer = current_time;
                fire_state = FIRE_PULL_DOWN_3;
            }
            break;

        //下拉滑块到底部扳机
        case FIRE_PULL_DOWN_3:
            loader_target_mode=LOAD_MODE_FOLLOW;
            target_deliver_angle=(POS_BOTTOM);
            if (is_deliver_at_target(5)) {
                state_timer = current_time;
                fire_state = FIRE_WAIT_BOTTOM_3;
                loader_target_mode=LOAD_MODE_PARAL;
            }
            break;    

        // 底部等待
        case FIRE_WAIT_BOTTOM_3:
            if ((current_time - state_timer) > put_delay) {
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
                loader_target_mode=LOAD_MODE_FULL_DOWN;
            }
            break;

        // 缓冲区等待
        case FIRE_WAIT_UP_3:
            if ((current_time - state_timer) > before_fire_delay) {
                state_timer = current_time;
                fire_state = FIRE_RELOAD_LOWER_3;
            }
            break;
        /*todo
        song
        这里可以和上面的缓冲区等待合并
        */
        // 升降机下降，装填下一发
        case FIRE_RELOAD_LOWER_3:
            if ((current_time - state_timer) >1) {
                state_timer = current_time;
                fire_state = FIRE_SHOOTING_3;
            }
            break;
        
        // 射击
        case FIRE_SHOOTING_3:
            servo_igniter_unlock; // 解锁扳机舵机，发射
            LOG_INFO("Dart Fired! Total Count: %d", Robot.Status.dart_count + 1);
            if ((current_time - state_timer) > after_fire_delay) {
                Robot.Status.dart_count++; // 计数+1
                servo_igniter_lock;
                state_timer= current_time;
                fire_state = FIRE_RELOAD_LIFT_4;
            }
            break;

        // 升降机上升，准备装填第四发
        case FIRE_RELOAD_LIFT_4:
            loader_target_mode=LOAD_MODE_UP;
            if ((current_time - state_timer) >loader_up_delay) {
                state_timer = current_time;
                fire_state = FIRE_RELOAD_RELEASE_4;
            }
            break;

        // 卡镖释放，转移下一发到发射区
        case FIRE_RELOAD_RELEASE_4:
            servo_transfomer_unlock; // 松开卡镖舵机，转移下一发到发射区
            if ((current_time - state_timer) >relapse_delay) {
                servo_transfomer_lock; // 重新卡住卡镖舵机
                state_timer = current_time;
                fire_state = FIRE_CALIBRATION_4;
                start_deliver_calibration();
            }
            break;

        // 校准滑块
        case FIRE_CALIBRATION_4:
            check_calibration_logic();
            if(is_calibrated()){
                state_timer = current_time;
                fire_state = FIRE_PULL_DOWN_4;
            }
            break;

        //下拉滑块到底部扳机
        case FIRE_PULL_DOWN_4:
            loader_target_mode=LOAD_MODE_FOLLOW;
            target_deliver_angle=(POS_BOTTOM);
            if (is_deliver_at_target(5)) {
                state_timer = current_time;
                fire_state = FIRE_WAIT_BOTTOM_4;
                loader_target_mode=LOAD_MODE_PARAL;
            }
            break;
        
        // 底部等待
        case FIRE_WAIT_BOTTOM_4:
            if ((current_time - state_timer) > put_delay) {
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
                loader_target_mode=LOAD_MODE_FULL_DOWN;
            }
            break;

        // 缓冲区等待
        case FIRE_WAIT_UP_4:
            if ((current_time - state_timer) > before_fire_delay) {
                state_timer = current_time;
                fire_state = FIRE_RELOAD_LOWER_4; // 完成发射，回到闲置状态
            }
            break;
        /*todo
        song
        这里可以和上面的缓冲区等待合并
        */
        // 升降机下降，装填第四发
        case FIRE_RELOAD_LOWER_4:
            if ((current_time - state_timer) >1) {
                state_timer = current_time;
                fire_state = FIRE_SHOOTING_4;
            }
            break;

        // 射击    
        case FIRE_SHOOTING_4:
            servo_igniter_unlock; // 解锁扳机舵机，发射
            LOG_INFO("Dart Fired! Total Count: %d", Robot.Status.dart_count + 1);
            if ((current_time - state_timer) > after_fire_delay) {
                Robot.Status.dart_count++; // 计数+1
                servo_igniter_lock;
                state_timer= current_time;
                fire_state = FIRE_IDLE; // 完成发射，回到闲置状态
                //每打4发,就需要将S1回中再下按,否则不会继续发射
                if(Robot.Status.dart_count%4==0){
                    Robot.Flag.Status.stop_continus_fire=true;
                    Robot.Status.current_state = SYS_STANDBY;
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
        LOG_INFO("Fire State Change: %d -> %d", last_fire_state,fire_state);
        last_fire_state = fire_state;
    }
}
