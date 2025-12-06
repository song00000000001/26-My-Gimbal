#include "internal.h"
#include "global_data.h"
#include "robot_config.h"
#include "tim.h"

/**
 * @brief 装填主控任务
 * @parma None
 * @return None
 */
void Loader_Ctrl(void *arg)
{
	/*	pre load for task	*/
	Motor_CAN_COB Tx_Buff = {};
	TickType_t xLastWakeTime_t;
    myPID anglepid;									// 装填电机的编码器环
    myPID speedpid;									// 装填电机的速度环
	xLastWakeTime_t = xTaskGetTickCount();
	anglepid.SetPIDParam(4, 0, 0.1, 100, 1200);
	speedpid.SetPIDParam(8, 1, 0, 100, 16000);
    static uint16_t goal=0;										// 装填电机的目标值
    bool open = 1;											// 此值为0时装填镖体，为1时准备发射
    int status = 1;						            // 装填状态（1~4分别对应4发飞镖）
	for (;;)
	{
		vTaskDelayUntil(&xLastWakeTime_t, 1);

        //装填舵机控制
        if (status != 0 && status % 4 == 1) // 第一发
		{
			if (open == 0)
			{
				goal = 2750;
			}
			Loader_Clamps_ClampAll();
        }
		if (status > 0 && status % 4 == 2) // 第二发
		{
			if (open == 0)
			{
				goal = 1340;
			}
            Loader_Clamps_Release1();
		}
		if (status > 0 && status % 4 == 3) // 第三发
		{
			if (open == 0)
			{
				goal = 4090;
			}
			Loader_Clamps_Release2();
		}
		if (status > 0 && status % 4 == 0) // 第四发
		{
			if (open == 0)
			{
				goal = 6820;
			}
			Loader_Clamps_Release3();
		}

        //装填电机控制
        anglepid.Target = goal;
        anglepid.Current = loadermotor[0].getAngle();
        anglepid.Adjust();
        speedpid.Target = anglepid.Out;
        speedpid.Current = loadermotor[0].getSpeed();
        speedpid.Adjust();
        //以下的输出直接修改out,并没有利用motor_current类的torquecurrent,就没有反电动势修正
        //loadermotor[0].Out = speedpid.Out;
        //改为调用setCurrentOut函数，利用反电动势补偿,测试
        loadermotor[0].setCurrentOut(speedpid.Out);

        //发送CAN报文
		motor_dji::MotorMsgPack(Tx_Buff, loadermotor[0]);
        xQueueSend(CAN2_TxPort, &Tx_Buff.Id1ff, 0);   
		//xQueueSend(CAN2_TxPort, &Tx_Buff.Id2ff, 0);    
	}
}

/**
 * @brief 装填电机的编码器双环控制函数
 * @parma None
 * @return None
 */
void turn1(float angle)
{
	
}
