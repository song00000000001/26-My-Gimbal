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
	xTaskCreate(Task_VofaMonitor, "tskVofaMonitor", Small_Stack_Size + Tiny_Stack_Size, NULL, PriorityRealtime, &VofaMonitor_Handle);
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

#if USE_SRML_VOFA_MONITOR
void Task_VofaMonitor(void *arg){
	/* Cache for Task */

	/* Pre-Load for task */
	TickType_t xLastWakeTime_t;
	xLastWakeTime_t = xTaskGetTickCount();

	/* Infinite loop */
	while(1)
	{
		/* Wait for the next cycle */
		vTaskDelayUntil(&xLastWakeTime_t, 5);

		/* 在此处传入需要观察的变量，第一个参数为通道的起始编号 */
		//VofaMonitor::setDatas(0, mpu_receive.pitch, mpu_receive.yaw, mpu_receive.roll);
		//VofaMonitor::setDatas(3, data3, data4, data5);
		//VofaMonitor::setDatas(6, data6, data7, data8, data9);
		/* 选择串口id */
        if(Debugger.enable_debug_mode==0) 
            continue;
        else if(Debugger.enable_debug_mode==4)
            VofaMonitor::setDatas(0,
            Launcher.DeliverMotor[0].getMotorTotalAngle(),Launcher.DeliverMotor[1].getMotorTotalAngle(),
            Launcher.pid_deliver_spd[0].Target,Launcher.pid_deliver_spd[1].Target,
            Launcher.pid_deliver_spd[0].Current,Launcher.pid_deliver_spd[1].Current);
        else if(Debugger.enable_debug_mode==5)
            VofaMonitor::setDatas(0,Launcher.pid_deliver_spd[0].Target,Launcher.pid_deliver_spd[0].Current,Launcher.pid_deliver_spd[0].Out);
        else if(Debugger.enable_debug_mode==6){
            VofaMonitor::setDatas(0,Launcher.pid_deliver_sync.Current,Launcher.pid_deliver_sync.Out,//同步pid的now和输出
            Launcher.pid_deliver_spd[0].Target,Launcher.pid_deliver_spd[0].Current,Launcher.pid_deliver_spd[0].Out,//左电机速度环的目标，now，输出
            Launcher.pid_deliver_spd[1].Target,Launcher.pid_deliver_spd[1].Current,Launcher.pid_deliver_spd[1].Out
            
			);
        }
        else if(Debugger.enable_debug_mode==7){
            VofaMonitor::setDatas(0, 
                // 1. 观察基础期望：原本想跑多少？
                Launcher.pid_deliver_pos[0].Out,       // [通道0] 基础位置环输出 (理论上左右差不多)
                
                // 2. 观察最终指令：算法让它跑多少？(观察是否被削峰，是否有差值)
                Launcher.pid_deliver_spd[0].Target,    // [通道1] 左电机最终速度目标
                Launcher.pid_deliver_spd[1].Target,    // [通道2] 右电机最终速度目标
                
                // 3. 观察实际响应：实际跑了多少？
                Launcher.DeliverMotor[0].getMotorSpeed(), // [通道3] 左实际速度
                Launcher.DeliverMotor[1].getMotorSpeed(), // [通道4] 右实际速度

                // 4. 观察同步误差：误差收敛情况
                Launcher.pid_deliver_sync.Current         // [通道5] 两电机位置差 (Diff)
            );
        }
		VofaMonitor::send(4);
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
