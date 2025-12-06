
#include "internal.h"
#include "global_data.h"

/**
 * @brief Yaw轴控制任务
 * @parma None
 * @return None
 */

 float Yaw_Angle[2]; // 默认前哨站和基地角度

void Yaw_Task(void *arg)
{
	Motor_CAN_COB Tx_Buff1;
	TickType_t xLastWakeTime_t;
	static uint32_t tick = xTaskGetTickCount();
	uint32_t currentTick = xTaskGetTickCount();
    float yaw_target = 0, yaw_goal = 0, igniter_target_pos = 0, igniter_goal_pos = 0;
    bool vision_aim_state = 0;				// 视觉瞄准状态
    float storage_base_angle; // 视觉基准角、原基准角暂存
    float _YawCorrectionAngle;//yaw轴修正角
	yaw_target = 0;
	for (;;)
	{
		vTaskDelayUntil(&xLastWakeTime_t, 1);
		if (DR16.GetStatus() == DR16_ESTABLISHED)
		{
			if (Yaw.is_Yaw_Init() == 1 && DR16.GetS1() == SW_UP) // 左拨杆朝上，进入调试模式
			{
				yaw_target -= DR16.Get_LX_Norm() * 0.002f;
				yaw_target = std_lib::constrain(yaw_target, -10.2f, 10.2f);
				Yaw.update(yaw_target);
			}
			if (Yaw.is_Yaw_Init() == 1 && DR16.GetS1() == SW_DOWN) // 左拨杆朝下拨，进入视觉接管模式
			{
				storage_base_angle = Yaw_Angle[HitTarget];
				if (Yaw.is_Yaw_Init() == 1 && DR16.GetS1() == SW_DOWN)
				{
					//			if(vision_recv_pack.ros==3)//若视觉未识别到引导灯，则先自行扫描
					//			{
					//			    if(scan==0&&yaw_target<=7.5)
					//             {yaw_target+=0.01;
					//						  if(yaw_target==7.5){scan=1;}
					//						 }
					//					if(scan==1&&yaw_target>=-7.5)
					//					   {yaw_target-=0.01;
					//						  if(yaw_target==-7.5){scan=2;}
					//						 }
					//					if(scan==2){yaw_target=7.5;}
					//			}
					if (vision_recv_pack.ros == 1)
					{
						yaw_target += 0.0003;
					}
					if (vision_recv_pack.ros == 2)
					{
						yaw_target -= 0.0003;
					}
					if (vision_recv_pack.ros == 0)
					{
						yaw_target += 0;
					}
					yaw_target = std_lib::constrain(yaw_target, -10.2f, 10.2f);
					Yaw.update(yaw_target);
					Yaw_Angle[HitTarget] = yaw_target;
				}
			}
			else if (Yaw.is_Yaw_Init() == 1 && (vision_aim_state == 1 || DR16.GetS1() == SW_MID)) // 非视觉模式  or 瞄准完成
			{
				Yaw.update(_YawCorrectionAngle + Yaw_Angle[HitTarget]); // 更改Yaw轴角度
			}
			if (Yaw.is_Yaw_Init() != 1)
			{
				Yaw.init();
			}
			/*关控保护*/
			if (DR16.GetStatus() != DR16_ESTABLISHED)
			{
				Yaw.disable();
			}
			Yaw.adjust();
			/*打包发送*/
			MotorMsgPack(Tx_Buff1, Yaw.YawMotor);
			xQueueSend(CAN1_TxPort, &Tx_Buff1.Id1ff, 0);
			//xQueueSend(CAN2_TxPort, &Tx_Buff1.Id1ff, 0);
		}
	}
}