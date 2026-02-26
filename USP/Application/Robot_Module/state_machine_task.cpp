#include "internal.h"
#include "global_data.h"
#include "motor_ctrl_driver.h"
#include "remote_ctrl_driver.h"
#include "robot_config.h"
#include "ws2812_ctrl_driver.h"
void R_light(light_color_enum color);
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
    g_SystemState.BE_Group = 0;
    g_SystemState.BE_State = BE_GENERATE_TARGET;
    g_SystemState.CurrentHitID = 0;
    g_SystemState.SE_Group = 0; // 小能量轮数
    g_SystemState.SE_State = SE_GENERATE_TARGET; // 小能量状态机

    for (;;)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        main_task_now = xTaskGetTickCount();
        
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
            state_machine_reset();
            R_light(g_TargetCtrl.TargetColor);
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
            LightArmors();
            vTaskDelay(2000); 
            g_TargetCtrl.target_mode = tar_stop; // 结束，回到待机
        }
        break;
        default:
            state_machine_reset();
        break;
        }
                
        //记录任务剩余栈空间
        #ifdef INCLUDE_uxTaskGetStackHighWaterMark
        //Stack_Remain.task_state_machine_stack_remain = uxTaskGetStackHighWaterMark(NULL);
        #endif
    }
}

void R_light(light_color_enum color){
    switch(color){
        case color_red:
        case color_hit_red:
            ws2312_show(255, 0, 0); // 红色
            break;
        case color_blue:
        case color_hit_blue:
            ws2312_show(0, 0, 255); // 蓝色
            break;
        case color_off:
        default:
            ws2312_show(0, 0, 0); // 关灯
            break;
    }
}

void state_machine_reset(){
    g_SystemState.BE_Group = 0;
    g_SystemState.BE_State = BE_GENERATE_TARGET;
    g_SystemState.BE_Targets[0] = 0;
    g_SystemState.BE_Targets[1] = 0;
    g_SystemState.CurrentHitID = 0;
    g_SystemState.SE_Group = 0; // 重置小能量轮数
    g_SystemState.SE_State = SE_GENERATE_TARGET; // 重置小能量状态机
    ResetArmors(); // 熄灭所有装甲板
    R_light(color_off);
}
