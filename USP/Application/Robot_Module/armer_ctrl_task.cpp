/* Robot_Module/armer_ctrl_task.cpp */

#include "internal.h"
#include "global_data.h"
#include "robot_config.h"

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
