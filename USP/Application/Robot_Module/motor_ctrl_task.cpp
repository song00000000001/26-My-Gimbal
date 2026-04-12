#include "internal.h"
#include "global_data.h"
#include "robot_config.h"
#include "comm_protocal.h"


static void motor_init(uint8_t port_id);
static void motor_disable();



void gimbal_pid_init(void)
{
    vTaskDelay(1000);
    for(uint8_t i=0;i<MOTOR_COUNT;i++){
        MyPid_Init(&gimbal_pid_pos[i], MY_PID_MODE_POSITION, 0.0f, 0.0f, 0.0f, 0.007f);//dt根据任务周期设置，这里是7ms
        // 输出为电机转速，电机空载转速400rpm，额定100rpm，极值参考驱动函数。
        MyPid_SetLimit(&gimbal_pid_pos[i],
                       -500.0f,   500.0f,      // target_accum / out 限幅
                       0.0f, 0.0f,       // 积分限幅
                       0,  0);              //特殊模式下的增量限幅，这里无意义
        MyPid_SetIntegSplitThreshold(&gimbal_pid_pos[i], 5.0f); // 误差超过5度时暂停并清空积分，避免大误差引起的积分风暴。
    }
}



/**
 * @brief 电机控制任务
 */

void task_motor_ctrl(void *arg)
{
    TickType_t xLastWakeTime_t;
    xLastWakeTime_t = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(motor_comm_delay_ms); // 7ms周期，确保电机通信正常
    motor_init(motor_uart_id); // 初始化电机
    gimbal_pid_init(); // 初始化PID控制器

    for (;;)
    {
        
        vTaskDelayUntil(&xLastWakeTime_t, xFrequency);
        
        for(int i=0;i<MOTOR_COUNT;i++){
            MyPid_Calc(&gimbal_pid_pos[i],hold_angle_deg[i],imu_angle_deg[i]);
        }
        
        gimbal_motors[YAW].sendSpeedCtrl(gimbal_pid_pos[YAW].data.out,Debugger.motor_accel_time,Debugger.motor_brake_enable); // 速度控制指令)

        #if STACK_REMAIN_MONITER_ENABLE
        StackWaterMark_Get(motor_ctrl);
        #endif
    }
}

/**
 * @brief 电机初始化
 * @param port_id 串口ID，例如 1 表示 USART1
 * @details 该函数负责电机的初始化配置，包括注册发送函数和设置串口ID。需要在使用电机之前调用一次，确保驱动内部能够正确发送指令包
 * @todo
 * 这里的4ms延时是因为电机最快通信周期为4ms。
 * 第一次发送指令后需要等4ms才能发送下一条指令，否则可能会被电机拒绝或者丢包。
 * 考虑在communication层按照4ms的周期从队列里取出指令包发送给电机。
 * 但是手册只写了单个电机的通信周期，两个电机同时占用总线通信时是否需要更长的周期还不确定，后续需要测试。
 */
static void motor_init(uint8_t port_id)
{
    vTaskDelay(pdMS_TO_TICKS(1200)); // 等待系统稳定
    // 电机初始化逻辑
    // 1. 注册发送函数指针，供驱动内部调用发送指令包
    BenMoMotor::registerSendFunction(send_motor_packet);
    // 2. 设置发送使用的串口ID
    for (int i = 0; i < MOTOR_COUNT; i++) {
        vTaskDelay(pdMS_TO_TICKS(10));

        gimbal_motors[i].setPortNum(port_id);// 设置串口ID
        vTaskDelay(pdMS_TO_TICKS(10));

        gimbal_motors[i].sendEnableCmd(true); // 使能
        vTaskDelay(pdMS_TO_TICKS(10));

        gimbal_motors[i].sendModeCmd(MotorMode::SPEED_LOOP); // 切换到速度环模式
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/**
 * @brief 电机失能
 * @details 该函数负责电机的失能配置，通常在系统关闭或者需要紧急停止电机时调用。需要确保在调用该函数后，电机不会再响应任何控制指令。
 */
static void motor_disable()
{
    for (int i = 0; i < MOTOR_COUNT; i++) {
        vTaskDelay(pdMS_TO_TICKS(10));
        gimbal_motors[i].sendEnableCmd(false); // 失能
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
