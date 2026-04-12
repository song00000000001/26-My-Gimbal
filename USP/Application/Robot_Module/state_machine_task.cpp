#include "internal.h"
#include "global_data.h"
#include "robot_config.h"

//状态机任务
void task_state_machine(void *arg)
{
    // 任务频率控制
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(2);
    uint32_t main_task_now = xTaskGetTickCount();
   
    for (;;)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        main_task_now = xTaskGetTickCount();

        //记录任务剩余栈空间
        #if STACK_REMAIN_MONITER_ENABLE
        //Stack_Remain.task_state_machine_stack_remain = uxTaskGetStackHighWaterMark(NULL);
        #endif
    }
}
