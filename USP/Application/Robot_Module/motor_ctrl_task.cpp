#include "internal.h"
#include "global_data.h"
#include "robot_config.h"
#include "comm_protocal.h"

void task_motor_ctrl(void *arg)
{
    TickType_t xLastWakeTime_t;
    xLastWakeTime_t = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(4);
    BenMo_Motor_1 = BenMoMotor(1); // 初始化电机1
    BenMo_Motor_1.genEnableCmd(true); // 使能电机1
    

    BenMo_Motor_2.genEnableCmd(true); // 使能电机2
    BenMo_Motor_2 = BenMoMotor(2); // 初始化电机2
    for (;;)
    {
        vTaskDelayUntil(&xLastWakeTime_t, xFrequency);// 4ms周期
        uint32_t time_clock = xTaskGetTickCount();// 实时更新时间戳

        

        #if STACK_REMAIN_MONITER_ENABLE
        StackWaterMark_Get(motor_ctrl);
        #endif
    }
}
