/* Robot_Module/loader_task.cpp */

#include "internal.h"
#include "global_data.h"
#include "robot_config.h"

// 线性映射函数：将 x 从 [in_min, in_max] 映射到 [out_min, out_max]
float map_f(float x, float in_min, float in_max, float out_min, float out_max) {
    // 限制输入范围，防止越界
    float input = std_lib::constrain(x, in_max,in_min);
    return (input - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
// 定义中间点：底部偏上的 PWM 值（预加载位置）
// 参考 global_data.cpp：Up1: 190, Down1: 75 | Up2: 100, Down2: 235
// 假设比最低点高 30 ccr
//uint8_t loader1_pre_down = servo_ccr.loader1_ccr_down + predown_ccr;  
typedef struct {
    uint8_t predown_ccr;
    uint8_t paral_down_ccr;
    uint8_t top_ccr_offset;
} loader_servo_params_t;

loader_servo_params_t loader_servo_params={
    20, //预加载位置偏移量
    10, //平行装填位置偏移量
    5   //顶端位置偏移量
};

/*装填控制任务*/
void Loader_Ctrl(void *arg)
{

	float current_pos=-20;
    uint8_t target_ccr1, target_ccr2;

    for (;;)
    {
        // 获取当前滑块位置
        // 修改位置获取逻辑：若处于模拟测试，则使用 simulated_loader_pos
        if (Debugger.is_loader_simulating) {
            current_pos = Debugger.simulated_loader_pos;
        } 
        else if (Debugger.enable_debug_mode == 1) {
            current_pos=Debugger.debug_loader_pos;
			if( current_pos>=POS_DELIVER_MAX)
                current_pos=POS_DELIVER_MAX;
            else if(current_pos<=POS_DELIVER_MIN)
                current_pos=POS_DELIVER_MIN;
        }
		else{
            current_pos = Launcher.DeliverMotor[0].getMotorTotalAngle();
		}
		
        switch (Launcher.loader_target_mode)
        {
        case LOAD_STOWED:
            target_ccr1 = servo_ccr.loader1_ccr_up+loader_servo_params.top_ccr_offset;
            target_ccr2 = servo_ccr.loader2_ccr_up-loader_servo_params.top_ccr_offset;
            break;

        case LOAD_DYNAMIC_SYNC:
            // 线性随动：当滑块从 POS_BUFFER 运动到 POS_BOTTOM 时
            // 舵机从 UP 位置 运动到 PRE_DOWN 位置
            target_ccr1 = (uint16_t)map_f(current_pos, POS_BUFFER, POS_BOTTOM, 
                                          servo_ccr.loader1_ccr_up, servo_ccr.loader1_ccr_down + loader_servo_params.predown_ccr);
            target_ccr2 = (uint16_t)map_f(current_pos, POS_BUFFER, POS_BOTTOM, 
                                          servo_ccr.loader2_ccr_up, servo_ccr.loader2_ccr_down - loader_servo_params.predown_ccr);
            break;

        case LOAD_PRE_LOAD:
            target_ccr1 = servo_ccr.loader1_ccr_down+loader_servo_params.paral_down_ccr;
            target_ccr2 = servo_ccr.loader2_ccr_down-loader_servo_params.paral_down_ccr;
            break;

        case LOAD_ENGAGED:
            target_ccr1 = servo_ccr.loader1_ccr_down;
            target_ccr2 = servo_ccr.loader2_ccr_down;
            break;
        default:
            break;
        }

        //日志记录装填子状态
        static Loader_Target_Mode_e last_loader_mode = LOAD_STOWED;
        if (last_loader_mode != Launcher.loader_target_mode) {
            #if enum_X_Macros_disable
            LOG_INFO("Loader target mode changed: %d -> %d", last_loader_mode, Launcher.loader_target_mode);
            #else
            LOG_INFO("Loader target mode changed: %s -> %s", Loader_Target_Mode_To_Str(last_loader_mode), Loader_Target_Mode_To_Str(Launcher.loader_target_mode));
            #endif
            last_loader_mode = Launcher.loader_target_mode;
        }

                
        //记录舵机参数变化
        static servo_ccr_debug last_servo_ccr = servo_ccr;
        if (memcmp(&last_servo_ccr, &servo_ccr, sizeof(servo_ccr_debug)) != 0) {
            LOG_WARN("Servo CCR Updated: igniter_unlock=%d, lock=%d;loader1_up=%d, down=%d;loader2_up=%d,down=%d;transf_lock=%d, unlock=%d",
                servo_ccr.igniter_ccr_unlock, servo_ccr.igniter_ccr_lock,
                servo_ccr.loader1_ccr_up, servo_ccr.loader1_ccr_down,
                servo_ccr.loader2_ccr_up, servo_ccr.loader2_ccr_down,
                servo_ccr.transfomer_ccr_lock, servo_ccr.transfomer_ccr_unlock);
            memcpy(&last_servo_ccr, &servo_ccr, sizeof(servo_ccr_debug));
        }

        // 更新硬件
        #if 1
        Launcher.loader_servo_1_ctrl(target_ccr1);
        Launcher.loader_servo_2_ctrl(target_ccr2);
        #else
        __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, target_ccr1);
        __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, target_ccr2);
        #endif
        vTaskDelay(pdMS_TO_TICKS(10)); // 100Hz 更新率足够平滑
    }
}
