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

TaskHandle_t DR16_Handle;
TaskHandle_t Rx_Referee_Handle;
TaskHandle_t LaunchCtrl_Handle;
TaskHandle_t Loader_Ctrl_Handle;
TaskHandle_t Vision_Task_Handle;
TaskHandle_t log_Handle;
TaskHandle_t protocol_status_monitor_Handle;
TaskHandle_t load_test_ctrl_Handle;
//TaskHandle_t Yaw_Task_Handle;
/* Private function declarations ---------------------------------------------*/
void tskDR16(void *arg);
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
void Service_Devices_Init(void)
{
	xTaskCreate(LaunchCtrl, "App.LaunchCtrl", Large_Stack_Size, NULL, PriorityAboveNormal, &LaunchCtrl_Handle);
	/*todo
	song
	增大任务优先级
	将dr16失联等实时性要求高的逻辑放到另一个任务中，防止主控制任务意外卡死(使用互斥锁或者vtaskdelay等阻塞函数)
	*/
	xTaskCreate(Vision_Task, "App.Vision_Task", Small_Stack_Size, NULL, PriorityAboveNormal, &Vision_Task_Handle);
	xTaskCreate(Loader_Ctrl, "App.Loader_Ctrl", Normal_Stack_Size, NULL, PriorityAboveNormal, &Loader_Ctrl_Handle);
    xTaskCreate(Task_load_test_ctrl, "App.load_test_ctrl", Normal_Stack_Size, NULL, PriorityNormal, &load_test_ctrl_Handle);

#if USE_SRML_DR16
	xTaskCreate(tskDR16, "App.DR16", Small_Stack_Size, NULL, PrioritySuperHigh, &DR16_Handle);
#endif
#if USE_SRML_REFEREE
	xTaskCreate(Rx_Referee, "Rx_Referee", Normal_Stack_Size, NULL, PriorityNormal, &Rx_Referee_Handle);
#endif
#if 1
    xTaskCreate(Task_LogTransmit,"log.tx_task",Normal_Stack_Size,NULL,PriorityLow,&log_Handle);
#elif 0
    xTaskCreate(Task_protocal_status_monitor,"protocal.status",Normal_Stack_Size,NULL,PriorityLow,&protocol_status_monitor_Handle);
#endif
}


 
/*
**************************************************************************************
*	函 数 名: StackOverflowTest
*	功能说明: 任务栈溢出测试
*	形    参: 无
*	返 回 值: 无
**************************************************************************************
*/
static void StackOverflowTest(void)
{
	int16_t i;
	uint8_t buf[4906];
	
	(void)buf; /* 防止警告 */
	
	/*
	  1. 为了能够模拟任务栈溢出，并触发任务栈溢出函数，这里强烈建议使用数组的时候逆着赋值。
	     因为对于M3和M4内核的MCU，堆栈生长方向是向下生长的满栈。即高地址是buf[2047], 低地址
	     是buf[0]。如果任务栈溢出了，也是从高地址buf[2047]到buf[0]的某个地址开始溢出。
	        因此，如果用户直接修改的是buf[0]开始的数据且这些溢出部分的数据比较重要，会直接导致
	     进入到硬件异常。
	  2. 栈溢出检测是在任务切换的时候执行的，我们这里加个延迟函数，防止修改了重要的数据导致直接
	     进入硬件异常。
	  3. 任务vTaskTaskUserIF的栈空间大小是2048字节，在此任务的入口已经申请了栈空间大小
		 ------uint8_t ucKeyCode;
	     ------uint8_t pcWriteBuffer[500];
	     这里再申请如下这么大的栈空间
	     -------int16_t i;
		 -------uint8_t buf[2048];
	     必定溢出。
	*/
    /*todo
    song
    为什么实测无法触发栈溢出钩子函数？每次都是直接硬件异常？
    */
	for(i = 4095; i >= 0; i--)
	{
		buf[i] = 0x55;
		Stack_Remain.Vision_Task_stack_remain = uxTaskGetStackHighWaterMark(NULL);
		//vTaskDelay(1);
	}
}


/**
 * @brief 电视通信任务
 * @parma None
 * @return None
 */
void Vision_Task(void *arg)
{
	USART_COB UART_pack;
	TickType_t xLastWakeTime_t;
	xLastWakeTime_t = xTaskGetTickCount();
	UART_pack.port_num = 1;
	UART_pack.address = (uint8_t *)&vision_send_pack;
	UART_pack.len = sizeof(vision_send_pack);

    

	for (;;)
	{
		vTaskDelayUntil(&xLastWakeTime_t, 10); // 100Hz 频率发送
		vision_send_pack.mode = 3;

        // if(Debugger.enable_debug_mode==7)
        // {
        //    StackOverflowTest();
        // }

		#if 0
		if (DR16.GetStatus() == DR16_ESTABLISHED && DR16.GetS1() == SW_DOWN) // 左拨杆拨到下，进入视觉模式
		{
			vision_send_pack.tracker_bit = 1;
			vision_send_pack.calibration_state = (DR16.GetS2() == SW_DOWN); // 右拨杆拨到下，开始标定
		}
		else
		{
			vision_send_pack.calibration_state = 0;
		}
		if (xTaskGetTickCount() - vision_last_recv_time > 150)
		{
			vision_recv_pack.target_mode = 0;
		}
		#endif
		//	 SRML_UART_Transmit_DMA(&UART_pack);

        #ifdef INCLUDE_uxTaskGetStackHighWaterMark
        Stack_Remain.Vision_Task_stack_remain = uxTaskGetStackHighWaterMark(NULL);
        #endif
        
	}
}
#if USE_SRML_DR16
/**
 *	@brief	Dr16 data receive task
 */
void tskDR16(void *arg)
{
	/* Cache for Task */
	USART_COB Rx_Package;
	/* Pre-Load for task */
	DR16.Check_Link(xTaskGetTickCount());
	/* Infinite loop */
	for (;;)
	{
		// 1. 等待数据（不要拿锁！）
        // 减少等待时间，20ms，这样Check_Link检查频率更高，反应更快,
        //dr16数据更新频率70hz,所以最快不要小于15ms即可。
        // Check_Link是在超时后才运行
        if (xQueueReceive(DR16_QueueHandle, &Rx_Package, 20) == pdPASS)
        {
            // 2. 收到数据，拿锁进行解析和更新
            xSemaphoreTake(DR16_mutex, portMAX_DELAY);
            DR16.DataCapture((DR16_DataPack_Typedef *)Rx_Package.address);
            xSemaphoreGive(DR16_mutex);
        }

        // 3. 无论有没有收到数据，都要检查是否超时
        // 这里需要拿锁，因为 Check_Link 可能会修改 Status，而其他任务可能会读取 Status
        xSemaphoreTake(DR16_mutex, portMAX_DELAY);
        DR16.Check_Link(xTaskGetTickCount());
        xSemaphoreGive(DR16_mutex);

        #ifdef INCLUDE_uxTaskGetStackHighWaterMark
        Stack_Remain.DR16_stack_remain = uxTaskGetStackHighWaterMark(NULL);
        #endif
	}
}
#endif

#if USE_SRML_REFEREE
/**
 * @brief  接受裁判系统数据
 * @param  None.
 * @return None.
 */
void Rx_Referee(void *arg)
{
	/* Preoad for task */
	USART_COB *referee_pack;
	TickType_t xLastWakeTime_t = xTaskGetTickCount();

	/* Infinite loop */
	for (;;)
	{
		if (xTaskNotifyWait(0x00000000, 0xFFFFFFFF, (uint32_t *)&referee_pack, portMAX_DELAY) == pdTRUE)
		{
			Referee.unPackDataFromRF((uint8_t *)referee_pack->address, referee_pack->len);
		}

        #ifdef INCLUDE_uxTaskGetStackHighWaterMark
        Stack_Remain.Rx_Referee_stack_remain = uxTaskGetStackHighWaterMark(NULL);
        #endif
	}
}
#endif

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
    // 报错、停机或记录日志
    LOG_ERROR("Stack Overflow in task: %s", pcTaskName);
    OpenLog.Send();
    while(1); 
}
