/**
 ******************************************************************************
 * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
 * @file missle_ctrl.cpp
 * @author lpk
 * @brief 镖架主控代码
 * @date 2025-3-9
 * @version 1.0
 * @par Change Log：
 * <table>
 * <tr><th>Date <th>Version <th>Author <th>Description
 * <tr><td>2019-06-12 <td> 1.0 <td>S.B. <td>Creator
 * </table>
 *
 ==============================================================================
 ##### How to use this driver #####
 ==============================================================================
 @note
 -#
 ******************************************************************************
 * @attention
 *
 * if you had modified this file, please make sure your code does not have many
 * bugs, update the version Number, write dowm your name and the date, the most
* important is make sure the users will have clear and definite understanding
 * through your new brief.
 *
 * <h2><center>&copy; Copyright (c) 2019 - ~, SCUT-RobotLab Development Team.
 * All rights reserved.</center></h2>
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "internal.h"
#include "global_data.h"
/* Private define ------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

TaskHandle_t FS_I6X_Handle;
TaskHandle_t Rx_Referee_Handle;
TaskHandle_t task_state_machine_Handle;
TaskHandle_t task_motor_ctrl_Handle;
TaskHandle_t Vision_Task_Handle;
TaskHandle_t log_Handle;
TaskHandle_t protocol_status_monitor_Handle;
TaskHandle_t armer_ctrl_Handle;

#if USE_SRML_MPU6050
TaskHandle_t tskIMU_Handle;
#endif

void Service_Devices_Init(void)
{
	xTaskCreate(task_state_machine, "App.task_state_machine", Large_Stack_Size, NULL, PriorityHigh, &task_state_machine_Handle);
    xTaskCreate(task_motor_ctrl, "App.task_motor_ctrl", Small_Stack_Size+Tiny_Stack_Size, NULL, PriorityAboveNormal, &task_motor_ctrl_Handle);
    //xTaskCreate(armer_ctrl_task, "App.armer_ctrl_task", Small_Stack_Size, NULL, PriorityAboveNormal, &armer_ctrl_Handle);
    #if USE_SRML_MPU6050
    xTaskCreate(task_imu, "App.tskIMU",  Small_Stack_Size+Tiny_Stack_Size, NULL, PriorityNormal, &tskIMU_Handle);
    #endif
}

/**
 * @brief MPU6050读取数据
 */
void task_imu(void *arg)
{
    /* Pre-Load for task */
    TickType_t xLastWakeTime_t;
    xLastWakeTime_t = xTaskGetTickCount();
	//由于角度数据经常在-180到180跳变，这里直接加180度偏移，变成0~360，方便后续的PID计算和调试观察
    
    for (;;)
    {
        /* wait for next circle */
        vTaskDelayUntil(&xLastWakeTime_t, 4); // 4ms周期，确保MPU6050数据读取正常

        /*  读取MPU6050数据 */
        vTaskSuspendAll();      //挂起其他任务，防止被打断
        taskDISABLE_INTERRUPTS();//关闭中断，若使用中断关闭，请确保SRML定时器的中断不受FreeRTOS管辖
        dmp_read_data(&mpu_receive);
        taskENABLE_INTERRUPTS();
        xTaskResumeAll();

        imu_angle_deg[PITCH] = mpu_receive.roll ;
        imu_angle_deg[YAW] = mpu_receive.yaw;
        imu_gyro_dps[PITCH] = mpu_receive.gyro[0];
        imu_gyro_dps[YAW] = mpu_receive.gyro[1];
    }
}

