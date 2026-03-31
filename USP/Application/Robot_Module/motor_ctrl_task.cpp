#include "internal.h"
#include "global_data.h"
#include "robot_config.h"

void task_motor_ctrl(void *arg)
{
    TickType_t xLastWakeTime_t;
    xLastWakeTime_t = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(2);

    #if dm_motor_ctrl_mode
    Motor_CAN_COB Tx_Buff = {};
    motor_ctrl.set_motor_mode(MODE_SPEED);
    motor_ctrl.mymotor_pid_spd.SetPIDParam(0.0,0,0,0,10);
    #else
    CAN_COB Tx_Buff = {};
    motor_ctrl.mymotor.bindCanQueueHandle(CAN1_TxPort); // 绑定CAN1发送队列
    #endif

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
