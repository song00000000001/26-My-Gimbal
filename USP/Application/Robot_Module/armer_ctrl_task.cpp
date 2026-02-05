/* Robot_Module/loader_task.cpp */

#include "internal.h"
#include "global_data.h"
#include "robot_config.h"

void Task_load_test_ctrl(void *arg)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1);

    for (;;)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
       
        #ifdef INCLUDE_uxTaskGetStackHighWaterMark
        //Stack_Remain.Loader_Ctrl_stack_remain = uxTaskGetStackHighWaterMark(NULL);
        #endif
    }
}
