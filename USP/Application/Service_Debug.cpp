/**
 **********************************************************************************
 * @file   : Service_Debug.cpp
 * @brief  : Debug support file.This file provides access ports to debug.
 **********************************************************************************
 *
 **/
/* Includes ------------------------------------------------------------------*/
#include "internal.h"
#include "global_data.h"
#include "robot_config.h"
/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
TaskHandle_t UpperMonitor_Handle;
TaskHandle_t VofaMonitor_Handle;
TaskHandle_t AsuwaveMonitor_Handle;
/* Private function declarations ---------------------------------------------*/
void Task_UpperMonitor(void *arg);
void Task_VofaMonitor(void *arg);
void Task_AsuwaveMonitor(void *arg);
/* Function prototypes -------------------------------------------------------*/
/**
 * @brief  Initialize debug service based on Asuwave.
 */
void Service_Debug_Init(void)
{
#if USE_SRML_UPPER_MONITOR
	xTaskCreate(Task_UpperMonitor, "tskUpperMonitor", Small_Stack_Size + Tiny_Stack_Size, NULL, PriorityRealtime, &UpperMonitor_Handle);
#endif

#if USE_SRML_VOFA_MONITOR
	xTaskCreate(Task_VofaMonitor, "tskVofaMonitor", Small_Stack_Size + Tiny_Stack_Size, NULL, PriorityAboveNormal, &VofaMonitor_Handle);
#endif

#if USE_SRML_ASUWAVE_MONITOR
	xTaskCreate(Task_AsuwaveMonitor, "tskAsuwaveMonitor", Small_Stack_Size + Tiny_Stack_Size, NULL, PriorityRealtime, &AsuwaveMonitor_Handle);		
#endif
}

#if USE_SRML_UPPER_MONITOR
void Task_UpperMonitor(void *arg)
{
	/* Cache for Task */

	/* Pre-Load for task */
	TickType_t xLastWakeTime_t;
	xLastWakeTime_t = xTaskGetTickCount();

	UpperMonitor::init(4);// 开启上位机修改变量功能，与发送无关，不使用也能照常发送
	// 分配编号给被修改的变量
	// UpperMonitor::bind_Modified_Var(0, ...);
	// UpperMonitor::bind_Modified_Var(1, ...);
	// UpperMonitor::bind_Modified_Var(2, ...);
	// UpperMonitor::bind_Modified_Var(3, ...);
	// UpperMonitor::bind_Modified_Var(4, ...);

	/* Infinite loop */
	for (;;)
	{
		/* Wait for the next cycle */
		vTaskDelayUntil(&xLastWakeTime_t, 5);

		/* 在此处传入需要观察的变量，第一个参数为通道的起始编号 */
		// UpperMonitor::setDatas(0, mpu_receive.pitch, mpu_receive.yaw, mpu_receive.roll);
		//UpperMonitor::setDatas(3, data3, data4, data5);
		//UpperMonitor::setDatas(6, data6, data7, data8, data9);
		/* 选择串口id */
		UpperMonitor::send(4);
	}
}
#endif /* USE_SRML_UPEER_MONITOR */

float temp_rate=1.0f;

#if USE_SRML_VOFA_MONITOR
void Task_VofaMonitor(void *arg){
	/* Cache for Task */

	/* Pre-Load for task */
	TickType_t xLastWakeTime_t;
    xLastWakeTime_t = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(motor_comm_delay_ms); // 7ms周期，确保电机通信正常
	/* Infinite loop */
	while(1)
	{
		vTaskDelayUntil(&xLastWakeTime_t, xFrequency);

        switch (Debugger.enable_debug_mode)
        {
        case debug_mpuvoda_monitor:
            VofaMonitor::setDatas(0,
                (float)mpu_receive.yaw,
                (float)mpu_receive.pitch,
                (float)mpu_receive.roll,
                (float)mpu_receive.accel[0],
                (float)mpu_receive.accel[1],
                (float)mpu_receive.accel[2],
                (float)mpu_receive.gyro[0],
                (float)mpu_receive.gyro[1],
                (float)mpu_receive.gyro[2],
                (float)mpu_receive.sensors
            );
            break;
        case debug_mtvofa_monitor:
            VofaMonitor::setDatas(0
                ,(float)gimbal_pid_pos[YAW].data.ref
                ,(float)gimbal_pid_pos[YAW].data.fdb
                ,(float)gimbal_pid_pos[YAW].data.out
                ,(float)imu_gyro_dps[YAW]//imu的角速度反馈，单位为度每秒
                ,(float)gimbal_motors[YAW].getCurrent()//输出正常电流值，用手转约34ma,静止/失能时偶尔有突发20~30ma的峰，不清楚原因。
                ,(float)gimbal_motors[YAW].getSpeed() //输出正常速度值，用手转最快22.3rpm,
                // ,(float)gimbal_motors[YAW].getTemp() //输出正常温度值，约为27
                // ,(float)gimbal_motors[YAW].getFaultCode() //测试正常输出0,还没遇到异常
                //以下是额外内容,需要额外调用函数
                // ,(float)gimbal_motors[YAW].getMode()
                // ,(float)gimbal_motors[YAW].getMileage()
                // ,(float)gimbal_motors[YAW].getPosition()
            );      
            break;   
        case debug_idle:
        default:
            break;
        }
         
		if(Debugger.enable_debug_mode != debug_idle)
		{
            VofaMonitor::send(6);
		}
			
        #if STACK_REMAIN_MONITER_ENABLE
        Stack_Remain.debug_send_stack_remain = uxTaskGetStackHighWaterMark(NULL);
        #endif
	}
}
#endif	/* USE_SRML_VOFA_MONITOR */

#if USE_SRML_ASUWAVE_MONITOR	
void Task_AsuwaveMonitor(void *arg){
	/* Cache for Task */

	/* Pre-Load for task */
	TickType_t xLastWakeTime_t;
	xLastWakeTime_t = xTaskGetTickCount();

	AsuwaveMonitor::init(1, xTaskGetTickCount);

	/* Infinite loop */
	while(1)
	{
		/* Wait for the next cycle */
		vTaskDelayUntil(&xLastWakeTime_t, 5);

		AsuwaveMonitor::send(); // 发送数据给上位机
	}
}
#endif /* USE_SRML_ASUWAVE_MONITOR */
/************************ COPYRIGHT SCUT-ROBOTLAB *****END OF FILE*************/
