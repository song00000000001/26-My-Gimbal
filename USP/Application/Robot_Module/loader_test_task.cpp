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
        uint32_t now = xTaskGetTickCount();

        // --- Buzzer Feedback Logic ---
        static bool is_beeping = false;
        static int beep_toggles_total = 0;
        static int beep_toggles_done = 0;
        static uint32_t beep_next_toggle_tick = 0;
        static uint32_t beep_interval_ticks = 0;

        if (Debugger.buzzer_beep_count > 0)
        {
            is_beeping = true;
            beep_toggles_total = Debugger.buzzer_beep_count * 2;
            
            // Interval to fit counts in 1000ms
            uint32_t interval_ms = 1000 / beep_toggles_total;
            if (interval_ms < 10) interval_ms = 10;
            
            beep_interval_ticks = pdMS_TO_TICKS(interval_ms);
            if (beep_interval_ticks == 0) beep_interval_ticks = 1;

            beep_next_toggle_tick = now + beep_interval_ticks;
            beep_toggles_done = 1;
            BEEP_ON;
            
            Debugger.buzzer_beep_count = 0;
        }
        else if (is_beeping)
        {
            if (now >= beep_next_toggle_tick)
            {
                beep_toggles_done++;
                if (beep_toggles_done > beep_toggles_total)
                {
                    BEEP_OFF;
                    is_beeping = false;
                }
                else
                {
                    if (beep_toggles_done % 2 != 0) BEEP_ON;
                    else BEEP_OFF;
                    
                    beep_next_toggle_tick = now + beep_interval_ticks;
                }
            }
        }
        // -----------------------------

        float ry_val = abs(DR16_Snap.RY_Norm);

        // --- 1. 触发逻辑判断 ---
        // 仅在测试未进行(test_step == 0)时响应触发
        if (test_step == 0 && ry_val > 0.5f && DR16_Snap.S1==SW_UP) 
        {
            if (DR16_Snap.S2 == SW_MID) {
                test_step = 1;      // 流程 A：单独模拟测试
                test_timer = now;
                LOG_INFO("Manual Loader Test Start (Individual Sim)");
            } 
            else if (DR16_Snap.S2 == SW_DOWN) {
                test_step = 11;     // 流程 B：卡镖 + 模拟测试
                test_timer = now;
                LOG_INFO("Manual Loader Test Start (Release + Sim)");
            }
        }

        // --- 2. 测试状态机 ---
        if (test_step > 0) 
        {
            switch (test_step) 
            {
                /* ======= 流程 A: 单独升降机模拟测试 (S2=Middle) ======= */
                case 1: // 模拟 2s 线性下降过程
                {
                    uint32_t elapsed = now - test_timer;
                    Launcher.loader_target_mode = LOAD_DYNAMIC_SYNC;
                    Debugger.is_loader_simulating = true; // 强制 Loader_Ctrl 使用模拟位置

                    if (elapsed < 2000) {
                        float t = (float)elapsed / 2000.0f;
                        // 从 POS_BUFFER (-20) 线性模拟到 POS_BOTTOM (-645)
                        Debugger.simulated_loader_pos = POS_BUFFER + t * (POS_BOTTOM - POS_BUFFER);
                    } else {
                        Debugger.simulated_loader_pos = POS_BOTTOM;
                        test_step = 2;
                        test_timer = now;
                    }
                }
                break;

                case 2: // 平行位置 (Preload) 延时 1s
                    Debugger.is_loader_simulating = false;
                    Launcher.loader_target_mode = LOAD_PRE_LOAD;
                    if (now - test_timer >  fire_sequence_delay_params.put_delay) {
                        test_step = 3;
                        test_timer = now;
                    }
                    break;

                case 3: // 到底 (Engaged) 停住 1s
                    Launcher.loader_target_mode = LOAD_ENGAGED;
                    if (now - test_timer >  fire_sequence_delay_params.before_fire_delay) {
                        test_step = 4;
                    }
                    break;

                case 4: // 返回顶部复位
                    Launcher.loader_target_mode = LOAD_STOWED;
                    test_step = 0; 
                    LOG_INFO("Loader Test A Finished.");
                    break;

                /* ======= 流程 B: 升降机 + 卡镖出仓测试 (S2=Down) ======= */
                case 11: // 降到底稳定 1s
                    Launcher.loader_target_mode = LOAD_ENGAGED;
                    if (now - test_timer >  fire_sequence_delay_params.after_fire_delay) {
                        test_step = 12;
                        test_timer = now;
                    }
                    break;

                case 12: // 直接上升到顶并延时 200ms
                    Launcher.loader_target_mode = LOAD_STOWED;
                    if (now - test_timer >  fire_sequence_delay_params.loader_up_delay) {
                        test_step = 13;
                        test_timer = now;
                    }
                    break;

                case 13: // 释放卡镖舵机 100ms
                    Launcher.servo_transfomer_unlock_f();
                    if (now - test_timer > fire_sequence_delay_params.relapse_delay) {
                        Launcher.servo_transfomer_lock_f();  
                        test_step = 14;
                        test_timer = now;
                    }
                    break;

                case 14: // 延时 2s 后启动下降流程
                    if (now - test_timer > 1000) {
                        test_step = 15;
                        test_timer = now;
                    }
                    break;

                case 15: // 衔接流程 A 的模拟下降 (2s)
                {
                    uint32_t elapsed = now - test_timer;
                    Launcher.loader_target_mode = LOAD_DYNAMIC_SYNC;
                    Debugger.is_loader_simulating = true;
                    
                    if (elapsed < 2000) {
                        float t = (float)elapsed / 2000.0f;
                        Debugger.simulated_loader_pos = POS_BUFFER + t * (POS_BOTTOM - POS_BUFFER);
                    } else {
                        Debugger.simulated_loader_pos = POS_BOTTOM;
                        test_step = 16;
                        test_timer = now;
                    }
                }
                break;

                case 16: // 衔接流程 A 的平行 1s
                    Debugger.is_loader_simulating = false;
                    Launcher.loader_target_mode = LOAD_PRE_LOAD;
                    if (now - test_timer > 1000) {
                        test_step = 17;
                    }
                    break;

                case 17: // 到底停住，不回升，等待下次触发
                    Launcher.loader_target_mode = LOAD_ENGAGED;
                    test_step = 0; 
                    LOG_INFO("Loader Test B Finished (Stopped at bottom).");
                    break;

                default:
                    test_step = 0;
                    Debugger.is_loader_simulating = false;
                    break;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10)); 
        #ifdef INCLUDE_uxTaskGetStackHighWaterMark
        Stack_Remain.Task_load_test_ctrl_stack_remain = uxTaskGetStackHighWaterMark(NULL);
        #endif
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
            ROBOT_LOG("CTRL", "Variable updated: %d (Beep: %d)", *target, Debugger.buzzer_beep_count);
        }
    }

    return delta;
}
