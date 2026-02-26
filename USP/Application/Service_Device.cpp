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
//TaskHandle_t Yaw_Task_Handle;
/* Private function declarations ---------------------------------------------*/

void armer_ctrl_task(void *arg);
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
	xTaskCreate(task_state_machine, "App.task_state_machine", Large_Stack_Size, NULL, PriorityHigh, &task_state_machine_Handle);
    xTaskCreate(task_motor_ctrl, "App.task_motor_ctrl", Small_Stack_Size+Tiny_Stack_Size, NULL, PriorityAboveNormal, &task_motor_ctrl_Handle);
    //xTaskCreate(armer_ctrl_task, "App.armer_ctrl_task", Small_Stack_Size, NULL, PriorityAboveNormal, &armer_ctrl_Handle);
}


