#include "internal.h"
#include "global_data.h"
#include "robot_config.h"

void task_motor_ctrl(void *arg)
{
    TickType_t xLastWakeTime_t;
    xLastWakeTime_t = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(2);

    for (;;)
    {
        vTaskDelayUntil(&xLastWakeTime_t, xFrequency);

        // 实时更新时间戳
        uint32_t time_clock = xTaskGetTickCount();

        

        #if STACK_REMAIN_MONITER_ENABLE
        StackWaterMark_Get(motor_ctrl);
        #endif
    }
}
