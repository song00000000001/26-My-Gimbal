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
TaskHandle_t load_test_ctrl_Handle;
//TaskHandle_t Yaw_Task_Handle;
/* Private function declarations ---------------------------------------------*/
void tskFS_I6X(void *arg);
void Rx_Referee(void *arg);
void Vision_Task(void *arg);
void Task_LogTransmit(void *arg);
void Task_protocal_status_monitor(void *arg);
void Task_load_test_ctrl(void *arg);
/* Function prototypes -------------------------------------------------------*/
/**
 * @brief  Initialization of device management service
 * @param  None.
 * @return None.
 */

/*todo
song
据分析，主任务优先级需要加大，防止控制周期抖动，但是主任务负担太重了，我比较担心会卡死。所以优先级还是保持吧。
后续优化的话，可以把一些实时性要求高的逻辑和电机控制放到另外的任务中，比如FS_I6X失联检测等。
然后通过无外设情况下的栈分析，任务FS_I6X和两个can发送的剩余栈分别只有71，56，71字节，有点少，后续可以适当增大这些任务的栈空间。

*/
void Service_Devices_Init(void)
{
    /*todo
    song
    改为
    1. 状态机主控任务
    2. 电机控制任务
    3. 装甲板控制任务
    4. 上位机通信任务
    5. 遥控器任务
    */
	xTaskCreate(task_state_machine, "App.task_state_machine", Large_Stack_Size, NULL, PriorityHigh, &task_state_machine_Handle);
	//xTaskCreate(Vision_Task, "App.Vision_Task", Small_Stack_Size+Tiny_Stack_Size, NULL, PriorityHigh, &Vision_Task_Handle);
	xTaskCreate(task_motor_ctrl, "App.task_motor_ctrl", Small_Stack_Size+Tiny_Stack_Size, NULL, PriorityAboveNormal, &task_motor_ctrl_Handle);
   // xTaskCreate(Task_load_test_ctrl, "App.load_test_ctrl", Small_Stack_Size+Tiny_Stack_Size, NULL, PriorityNormal, &load_test_ctrl_Handle);

#if USE_SRML_FS_I6X
	//xTaskCreate(tskFS_I6X, "App.FS_I6X", Small_Stack_Size+Tiny_Stack_Size, NULL, PrioritySuperHigh, &FS_I6X_Handle);
#endif
}

/**
 * @brief 电视通信任务
 * @parma None
 * @return None
 */
void Vision_Task(void *arg)
{
	TickType_t xLastWakeTime_t;
	xLastWakeTime_t = xTaskGetTickCount();

	for (;;)
	{
		vTaskDelayUntil(&xLastWakeTime_t, 100); // 10Hz 频率发送
		#if 1
		if (xTaskGetTickCount() - vision_last_recv_time > 150)
		{
		}
        else{
        }
		#endif
        #ifdef INCLUDE_uxTaskGetStackHighWaterMark
        //Stack_Remain.Vision_Task_stack_remain = uxTaskGetStackHighWaterMark(NULL);
        #endif
	}
}
#if USE_SRML_FS_I6X
/**
 *	@brief	FS_I6X data receive task
 */
void tskFS_I6X(void *arg)
{
	/* Cache for Task */
	USART_COB Rx_Package;
	/* Pre-Load for task */
	FS_I6X.Check_Link(xTaskGetTickCount());
	/* Infinite loop */
	for (;;)
	{
		// 1. 等待数据（不要拿锁！）
        // 减少等待时间，20ms，这样Check_Link检查频率更高，反应更快,
        //FS_I6X数据更新频率70hz,所以最快不要小于15ms即可。
        // Check_Link是在超时后才运行
        if (xQueueReceive(FS_I6X_QueueHandle, &Rx_Package, 20) == pdPASS)
        {
            // 2. 收到数据，拿锁进行解析和更新
            //xSemaphoreTake(FS_I6X_mutex, portMAX_DELAY);
            FS_I6X.DataCapture((SBUS_DataPack_Typedef*)Rx_Package.address);
            FS_I6X.DataProcess();
            FS_I6X.Check_Link(xTaskGetTickCount());
            //xSemaphoreGive(FS_I6X_mutex);
        }
        else{
            // 3. 无论有没有收到数据，都要检查是否超时
            // 这里需要拿锁，因为 Check_Link 可能会修改 Status，而其他任务可能会读取 Status
            //xSemaphoreTake(FS_I6X_mutex, portMAX_DELAY);
            FS_I6X.Check_Link(xTaskGetTickCount());
            //xSemaphoreGive(FS_I6X_mutex); 
        }
        #ifdef INCLUDE_uxTaskGetStackHighWaterMark
        Stack_Remain.FS_I6X_stack_remain = uxTaskGetStackHighWaterMark(NULL);
        #endif
	}
}
#endif
