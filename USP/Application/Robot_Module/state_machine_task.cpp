#include "internal.h"
#include "global_data.h"
#include "motor_ctrl_driver.h"
#include "remote_ctrl_driver.h"
#include "robot_config.h"
#include "ws2812_ctrl_driver.h"

void state_machine_reset();

//状态机任务
void task_state_machine(void *arg)
{
    // 任务频率控制
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(2);
    uint32_t main_task_now = xTaskGetTickCount();

    Motor_CAN_COB Tx_Buff = {};

    Debugger={
        .enable_debug_mode=debug_idle,
    };
     // --- 初始化部分 ---
    // 启动 ADC DMA (只需要启动一次)
    static uint32_t adcSeed = 0;
    adc_start(&adcSeed);
    
    // 稍微延时一下，确保 DMA 已经搬运了至少一次数据到 adcSeed
    // (因为 DMA 启动到完成第一次传输需要一点点时间，虽然极短)
    vTaskDelay(5); 

    // 设置随机数种子 (只需要执行一次)
    // 结合 ADC 悬空值和当前时间戳，保证每次上电的随机序列都不同
    srand(adcSeed + xTaskGetTickCount()); 

    g_SystemState.SysMode=small_energy; //默认小能量机关模式
    g_SystemState.BE_StateData.BE_Group = 0;
    g_SystemState.BE_StateData.BE_State = BE_GENERATE_TARGET;
    g_SystemState.CurrentHitID = 0;
    g_SystemState.SE_StateData.SE_Group = 0; // 小能量轮数
    g_SystemState.SE_StateData.SE_State = SE_GENERATE_TARGET; // 小能量状态机
    g_TargetCtrl={
        .target_mode = tar_small_energy_signle,        // 默认停止/待机
        .TargetColor = color_red,      // 默认红色
        .SmallEnergy_Speed = 1.0f,     // 默认倍率
        .BigEnergy_A = 0.9125f,        // 默认大符参数 (0.780 + 1.045)/2
        .BigEnergy_W = 1.942f          // 默认大符参数 (1.884 + 2.000)/2
    };
    
    for (;;)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        main_task_now = xTaskGetTickCount();
        
        debug_simulate_hit_f(); // 调试函数，模拟击打事件
        
        if(g_TargetCtrl.target_mode == tar_stop) // 停止/待机
            g_SystemState.SysMode=idle;
        else if(g_TargetCtrl.target_mode == tar_start) // 激活
            g_SystemState.SysMode=wait_start;
        
        // 模式判断
        switch (g_SystemState.SysMode)
        {
        case idle:
        {
            state_machine_reset();
            if(g_TargetCtrl.target_mode == tar_stop)
                g_SystemState.SysMode=idle;
            else
                g_SystemState.SysMode=wait_start;   
        }
        break;
        case wait_start:
        {
            //state_machine_reset();
            g_SystemState.TargetSpeed = 0.1f; // 初始目标速度

            switch (g_TargetCtrl.target_mode)
            {
            case tar_stop: // 停止/待机
                g_SystemState.SysMode=idle;
                break;
            case tar_start: // 激活
                g_SystemState.SysMode=wait_start;
                break;
            case tar_small_energy_signle: // 小能量机关
            case tar_small_energy_continue: // 连续小能量机关
                g_SystemState.SysMode = small_energy; // 切换到小能量机关
                break;
            case tar_big_energy_single: // 大能量机关
            case tar_big_energy_continue: // 连续大能量机关
                g_SystemState.SysMode = big_energy; // 切换到大能量机关
                break;
            default:
                break;
            }
        }
        break;
        case small_energy:
        {
            small_energy_logic();
        }
        break;
        case big_energy:
        {
            big_energy_logic();
        }
        break;
        case success:
        {
            // 全部点亮
            Ctrl_All_Armors(FAN_CMD_HIT, g_TargetCtrl.TargetColor, 5);
            vTaskDelay(2000); 
            g_TargetCtrl.target_mode = tar_stop; // 结束，回到待机
        }
        break;
        default:
            state_machine_reset();
        break;
        }
                
        //记录任务剩余栈空间
        #ifdef STACK_REMAIN_MONITER_ENABLE
        //Stack_Remain.task_state_machine_stack_remain = uxTaskGetStackHighWaterMark(NULL);
        #endif
    }
}

void state_machine_reset(){
    g_SystemState.BE_StateData.BE_Group = 0;
    g_SystemState.BE_StateData.BE_State = BE_GENERATE_TARGET;
    g_SystemState.BE_StateData.BE_Targets[0] = 0;
    g_SystemState.BE_StateData.BE_Targets[1] = 0;
    g_SystemState.CurrentHitID = 0;
    g_SystemState.SE_StateData.SE_Group = 0; // 重置小能量轮数
    g_SystemState.SE_StateData.SE_State = SE_GENERATE_TARGET; // 重置小能量状态机
    Ctrl_All_Armors(FAN_CMD_RESET, color_off, 0); // 熄灭所有装甲板
    //R_light(color_off);
}

void debug_simulate_hit_f() {
    if (Debugger.Debug_simulate_hit) {
        Debugger.Debug_simulate_hit = false; // 重置模拟击打标志
        if(g_SystemState.SysMode == small_energy && g_SystemState.SE_StateData.SE_State == SE_WAIT_HIT) {
            g_SystemState.CurrentHitID = g_SystemState.SE_StateData.SE_TargetID; // 模拟击中目标
        }
        else if(g_SystemState.SysMode == big_energy && (g_SystemState.BE_StateData.BE_State == BE_WAIT_HIT_1 || g_SystemState.BE_StateData.BE_State == BE_WAIT_HIT_2)) {
            if(g_SystemState.BE_StateData.BE_State == BE_WAIT_HIT_1) {
                g_SystemState.CurrentHitID = g_SystemState.BE_StateData.BE_Targets[0]; // 模拟击中第一个目标
            }
            else {
                g_SystemState.CurrentHitID = g_SystemState.BE_StateData.BE_Targets[1]; // 模拟击中第二个目标
            }
        }
        g_SystemState.CurrentHitScores = rand() % 10 + 1; // 模拟得分为1-10之间的随机数
    }
}