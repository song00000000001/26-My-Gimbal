/* Robot_Module/armer_ctrl_task.cpp */

#include "internal.h"
#include "global_data.h"
#include "robot_config.h"
#include "can_comm_protocal.h"  

void armer_ctrl_task(void *arg)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(2);

    for (;;)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
       
        #ifdef INCLUDE_uxTaskGetStackHighWaterMark
        //Stack_Remain.Armer_Ctrl_stack_remain = uxTaskGetStackHighWaterMark(NULL);
        #endif
    }
}


void LightArmors() {
    for (int i = 1; i <= 5; i++) {
        // 全部点亮
        SendFanPacket(i,FAN_CMD_HIT,g_TargetCtrl.TargetColor, g_SystemState.BE_Group);
    }
}

void ResetArmors() {
    for (int i = 1; i <= 5; i++) {
        // 全部熄灭
        SendFanPacket(i,FAN_CMD_RESET,color_off, g_SystemState.BE_Group);
    }
}

// 成功后闪烁重置,num>0正闪亮灭,num<0反闪灭亮
void lightSuccessFlash(int8_t num) {
    if(num < 0) {
        num = -num;
        for (uint8_t flash = 0; flash < num; flash++) {
            // 全部熄灭
            ResetArmors();
            vTaskDelay(500);
            // 全部点亮
            LightArmors();
            vTaskDelay(500);
        }
    }
    else if(num > 0) {
        for (uint8_t flash = 0; flash < num; flash++) {
            // 全部点亮
            LightArmors();
            vTaskDelay(500);
            // 全部熄灭
            ResetArmors();
            vTaskDelay(500);
        }
    }
    else 
        return;
}   