/* Robot_Module/loader_task.cpp */

#include "internal.h"
#include "global_data.h"
#include "robot_config.h"

float map_f(float x, float in_min, float in_max, float out_min, float out_max);

// 定义中间点：底部偏上的 PWM 值（预加载位置）
// 参考 global_data.cpp：Up1: 190, Down1: 75 | Up2: 100, Down2: 235
// 假设比最低点高 30 ccr
//uint8_t loader1_pre_down = servo_ccr.loader1_ccr_down + predown_ccr;  
uint8_t predown_ccr=30;
uint8_t paral_down_ccr=20;


/*装填控制任务*/
void Loader_Ctrl(void *arg)
{

	float current_pos=-20;
    uint8_t target_ccr1, target_ccr2;

    for (;;)
    {
        if (Debugger.enable_debug_mode!=1)
            current_pos = Launcher.DeliverMotor[0].getMotorTotalAngle();
		else{

			if( current_pos>=POS_DELIVER_MAX)
                current_pos=POS_DELIVER_MAX;
            else if(current_pos<=POS_DELIVER_MIN)
                current_pos=POS_DELIVER_MIN;
		}
		
        switch (Launcher.loader_target_mode)
        {
        case LOAD_MODE_UP:
            target_ccr1 = servo_ccr.loader1_ccr_up;
            target_ccr2 = servo_ccr.loader2_ccr_up;
            break;

        case LOAD_MODE_FOLLOW:
            // 线性随动：当滑块从 POS_BUFFER 运动到 POS_BOTTOM 时
            // 舵机从 UP 位置 运动到 PRE_DOWN 位置
            target_ccr1 = (uint16_t)map_f(current_pos, POS_BUFFER, POS_BOTTOM, 
                                          servo_ccr.loader1_ccr_up, servo_ccr.loader1_ccr_down + predown_ccr);
            target_ccr2 = (uint16_t)map_f(current_pos, POS_BUFFER, POS_BOTTOM, 
                                          servo_ccr.loader2_ccr_up, servo_ccr.loader2_ccr_down - predown_ccr);
            break;

        case LOAD_MODE_PARAL:
            target_ccr1 = servo_ccr.loader1_ccr_down+paral_down_ccr;
            target_ccr2 = servo_ccr.loader2_ccr_down-paral_down_ccr;
            break;

        case LOAD_MODE_FULL_DOWN:
            target_ccr1 = servo_ccr.loader1_ccr_down;
            target_ccr2 = servo_ccr.loader2_ccr_down;
            break;
        default:
            break;
        }

        //日志记录装填子状态
        static Loader_Target_Mode_e last_loader_mode = LOAD_MODE_UP;
        if (last_loader_mode != Launcher.loader_target_mode) {
            LOG_INFO("Loader target mode changed: %d -> %d", last_loader_mode, Launcher.loader_target_mode);
            last_loader_mode = Launcher.loader_target_mode;
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


// 线性映射函数：将 x 从 [in_min, in_max] 映射到 [out_min, out_max]
float map_f(float x, float in_min, float in_max, float out_min, float out_max) {
    // 限制输入范围，防止越界
    float input = std_lib::constrain(x, in_max,in_min);
    return (input - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}