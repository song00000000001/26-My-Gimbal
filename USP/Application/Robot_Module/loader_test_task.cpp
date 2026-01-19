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
                    if (now - test_timer > 100) {
                        Launcher.servo_transfomer_lock_f();  
                        test_step = 14;
                        test_timer = now;
                    }
                    break;

                case 14: // 延时 2s 后启动下降流程
                    if (now - test_timer > 2000) {
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
    }
}