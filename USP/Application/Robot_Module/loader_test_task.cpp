/* Robot_Module/loader_task.cpp */

#include "internal.h"
#include "global_data.h"
#include "robot_config.h"

void Task_load_test_ctrl(void *arg)
{
    static uint8_t test_step = 0;
    static uint32_t test_timer = 0;

    for (;;)
    {
        
    }
}

/**
 * @brief 摇杆步进控制函数（带蜂鸣器反馈）
 * @param input     当前摇杆的归一化输入 (-1.0 到 1.0)
 * @param state     指向该摇杆对应的状态结构体
 * @param target    指向要修改的变量
 * @param min_val   变量最小值
 * @param max_val   变量最大值
 */
int Step_Control_With_Feedback(float input, JoystickTrigger_t *state, int *target, int min_val, int max_val) {
    const float threshold = 0.85f; // 触发阈值
    int delta = 0;

    // --- 1. 边缘检测逻辑 ---
    
    // 正向触发检测 (从 0 到 1 的瞬间)
    bool current_pushed_pos = (input > threshold);
    if (current_pushed_pos && !state->last_pushed_pos) {
        delta = 1;
    }
    state->last_pushed_pos = current_pushed_pos;

    // 负向触发检测 (从 0 到 -1 的瞬间)
    bool current_pushed_neg = (input < -threshold);
    if (current_pushed_neg && !state->last_pushed_neg) {
        delta = -1;
    }
    state->last_pushed_neg = current_pushed_neg;

    // --- 2. 变量修改与反馈 ---
    if (delta != 0) {
        int next_val = *target + delta;

        // 边界检查
        if (next_val >= min_val && next_val <= max_val) {
            *target = next_val;

            // 触发蜂鸣器反馈
            // 规则：响声次数 = 当前数值 (如果数值为0，响1声短促音提示有效)
            if (*target == 0) {
                Debugger.buzzer_beep_count = 1; 
            } else {
                Debugger.buzzer_beep_count = (uint8_t)(*target);
            }
            
            // 打印日志方便调试
            LOG_INFO("CTRL Variable updated: %d (Beep: %d)", *target, Debugger.buzzer_beep_count);
        }
    }

    return delta;
}
