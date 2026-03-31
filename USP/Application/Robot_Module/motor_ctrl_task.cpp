#include "internal.h"
#include "global_data.h"
#include "robot_config.h"
#include "comm_protocal.h"

/**
 * @brief 电机初始化
 * @param port_id 串口ID，例如 1 表示 USART1
 * @details 该函数负责电机的初始化配置，包括注册发送函数和设置串口ID。需要在使用电机之前调用一次，确保驱动内部能够正确发送指令包
 * @todo
 * 这里的4ms延时是因为电机最快通信周期为4ms。
 * 第一次发送指令后需要等4ms才能发送下一条指令，否则可能会被电机拒绝或者丢包。
 * 后续考虑塞进类内部，自动处理发送节奏。但是可能会增加一些复杂度，比如需要在类内部维护一个定时器或者上次发送的时间戳，来判断是否可以发送下一条指令。
 * 甚至还要内部实现队列来缓存，这样就和FreeRTOS的任务调度机制有点重叠了，需要仔细设计一下。
 * 目前先放在外部，后续如果发现频繁发送指令导致问题，再考虑封装进类内部自动处理发送节奏。    
 */
static void motor_init(uint8_t port_id)
{
    // 电机初始化逻辑
    // 1. 注册发送函数指针，供驱动内部调用发送指令包
    BenMoMotor::registerSendFunction(send_motor_packet);
    // 2. 设置发送使用的串口ID
    for (int i = 0; i < MOTOR_COUNT; i++) {
        gimbal_motors[i].setPortNum(port_id);// 设置串口ID
        vTaskDelay(pdMS_TO_TICKS(4));
        gimbal_motors[i].genEnableCmd(true); // 使能
        vTaskDelay(pdMS_TO_TICKS(4));
        gimbal_motors[i].genModeCmd(MotorMode::POSITION_LOOP); // 切换到位置环模式
        vTaskDelay(pdMS_TO_TICKS(4));
        gimbal_motors[i].genPositionCtrl(0); // 位置控制指令，目标位置为 180° (16384 对应 180°)
        vTaskDelay(pdMS_TO_TICKS(4));
    }
}

/**
 * @brief 电机失能
 * @details 该函数负责电机的失能配置，通常在系统关闭或者需要紧急停止电机时调用。需要确保在调用该函数后，电机不会再响应任何控制指令。
 */
void motor_disable()
{
    for (int i = 0; i < MOTOR_COUNT; i++) {
        gimbal_motors[i].genEnableCmd(false); // 失能
        vTaskDelay(pdMS_TO_TICKS(4));
    }
}

/**
 * @brief 电机控制任务
 * 
 */
void task_motor_ctrl(void *arg)
{
    TickType_t xLastWakeTime_t;
    xLastWakeTime_t = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(4);
    motor_init(3); // 初始化电机，使用 USART3 进行通信
   
   
    for (;;)
    {
        vTaskDelayUntil(&xLastWakeTime_t, xFrequency);// 4ms周期
        uint32_t time_clock = xTaskGetTickCount();// 实时更新时间戳

        

        #if STACK_REMAIN_MONITER_ENABLE
        StackWaterMark_Get(motor_ctrl);
        #endif
    }
}
