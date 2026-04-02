#include "internal.h"
#include "global_data.h"
#include "robot_config.h"
#include "comm_protocal.h"


static void motor_init(uint8_t port_id);
static void motor_disable();



void gimbal_pid_init(void)
{
    vTaskDelay(10000);
    for(uint8_t i=0;i<MOTOR_COUNT;i++){
        MyPid_Init(&gimbal_pid[i], MY_PID_MODE_GIMBAL_INC_POS, 0.08f, 0.0f, 0.015f, 0.014f);

        // 输出直接就是位置目标，范围按电机 0~360°
        MyPid_SetLimit(&gimbal_pid[i],
                       0.0f,   360.0f,   // target_accum / out 限幅
                       -10.0f, 10.0f,    // 积分限幅
                       -2.0f,  2.0f);    // 每次位置增量限幅

        MyPid_SetAccumTarget(&gimbal_pid[i], imu_angle_deg[i]); // 初始目标位置
    }
}


#define motor_comm_delay_ms 7 // 电机通信周期，38400波特率下理论上4ms，但是发送可以保证4ms，如果要等电机回复需要6~7ms不等。
//7ms下测试丢包率1%~2%
//8ms下测试丢包率0.5%~1%
//认为7ms是一个比较合理的选择，既能保证通信效率又能降低丢包率。后续可以考虑增加重试机制来进一步降低丢包率。
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
        

        
        for(int i=0;i<MOTOR_COUNT;i++){
            motor_cmd_deg[i] = MyPid_CalcGimbal(&gimbal_pid[i],
                                       hold_angle_deg[i],
                                       imu_angle_deg[i],
                                       imu_gyro_dps[i]);
                                    
            vTaskDelayUntil(&xLastWakeTime_t, xFrequency);
            gimbal_motors[i].sendQueryExtraCmd(); // 请求里程和精确位置等额外反馈
            vTaskDelayUntil(&xLastWakeTime_t, xFrequency);
            gimbal_motors[i].sendPositionCtrl(motor_cmd_deg[i]); // 位置控制指令，目标位置由全局变量 motor_target_position 提供，单位为度
        }
        
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
        gimbal_motors[i].setPortNum(port_id);// 设置串口ID
        vTaskDelay(pdMS_TO_TICKS(motor_comm_delay_ms));
        gimbal_motors[i].sendEnableCmd(true); // 使能
        vTaskDelay(pdMS_TO_TICKS(motor_comm_delay_ms));
        gimbal_motors[i].sendModeCmd(MotorMode::POSITION_LOOP); // 切换到位置环模式
        vTaskDelay(pdMS_TO_TICKS(motor_comm_delay_ms));
        gimbal_motors[i].sendPositionCtrl(0); // 位置控制指令，目标位置为 180° (16384 对应 180°)
        vTaskDelay(pdMS_TO_TICKS(motor_comm_delay_ms));
    }
}

/**
 * @brief 电机失能
 * @details 该函数负责电机的失能配置，通常在系统关闭或者需要紧急停止电机时调用。需要确保在调用该函数后，电机不会再响应任何控制指令。
 */
static void motor_disable()
{
    for (int i = 0; i < MOTOR_COUNT; i++) {
        gimbal_motors[i].sendEnableCmd(false); // 失能
        vTaskDelay(pdMS_TO_TICKS(motor_comm_delay_ms));
    }
}
