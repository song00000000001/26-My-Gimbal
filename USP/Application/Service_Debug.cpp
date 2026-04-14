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

        if(!Debugger.system_enable){
            continue;
        }

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
                //,(float)imu_gyro_dps[YAW]//imu的角速度反馈，单位为度每秒
                ,(float)gimbal_pid_spd[YAW].data.ref
                ,(float)gimbal_pid_spd[YAW].data.fdb
                ,(float)gimbal_pid_spd[YAW].data.out
                ,(float)gimbal_pid_cur[YAW].data.ref
                ,(float)gimbal_pid_cur[YAW].data.fdb
                ,(float)gimbal_pid_cur[YAW].data.out
            );      
            /**
             * 关于响应
             * 测试发现电机的速度反馈响应最快，其次是imu的角速度反馈（相对上一个延时约42ms），其次是imu的角度反馈（42ms），其次是电机的电流反馈（84ms）。
             * 电机实际模式是电流环模式。因为使能后默认电流环，而代码是先设模式再使能，所以模式设置失效，而电流环和速度环共用寄存器指令，所以输出为电流环模式。然后数据还存在一定的偏差。需要修复。
             * 电流环模式下，12v电池供电，空载最大转速170rpm，最大堵转电流570ma。
             * 角度环模式下，12v电池供电，getspeed变成了编码器数据，范围是-1638.4~1638.4，对应-180°~180°，倒是省去了获取额外数据的步骤。但是是非常规，而且角度环模式没啥用。响应很慢。
             * 然后角度环猛转输出有650ma，但是转一点点等积分作用只有570ma。
             * 速度环模式下，12v电池供电，空载最大转速170rpm，控制时最大输出30,最大反馈电流336ma。响应一般,100ms左右。
             * 堵转最大电流573ma,反向旋转最大940ma
             * 而且角度环KP大了会莫名刹车，像是kd大了，但是我没给，应该是电机速度环模式的问题。
             * kp小了后，电机刹不住车，会触底反弹，一般震荡两次回落。
             */
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
