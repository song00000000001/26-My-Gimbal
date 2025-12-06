
#include "internal.h"
#include "global_data.h"

/**
 * @brief Yaw轴控制任务
 * @parma None
 * @return None
 */

float Yaw_Angle[2]; // 默认前哨站和基地角度

enum vision_aim_state_enum
{
    MANUAL_AIM = 0,
    VISION_AIM = 1
};

void Yaw_Task(void *arg)
{
	Motor_CAN_COB Tx_Buff1;
	TickType_t xLastWakeTime_t;

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
            //如果已经校准过
            if (Yawer.is_Yaw_Init() == 1){
                if(DR16.GetS2() == SW_UP)// 右拨杆朝上，不使能电机
                {
                    Yawer.disable();
                    //continue;
                }
                else if(DR16.GetS2() == SW_MID) // 右拨杆中档，手动模式
                {
                    vision_aim_state = MANUAL_AIM;
                }
                else if(DR16.GetS2() == SW_DOWN) // 右拨杆朝下，视觉模式
                {
                    vision_aim_state = VISION_AIM;
                }

                if(vision_aim_state == MANUAL_AIM)
                {
                    //_YawCorrectionAngle += DR16.Get_RY_Norm() * 0.1f; // 手动微调
                    //_YawCorrectionAngle = std_lib::constrain(_YawCorrectionAngle, -10.2f, 10.2f);
                    yaw_target -= DR16.Get_LX_Norm() * 0.002f;
                    yaw_target = std_lib::constrain(yaw_target, -10.2f, 10.2f);
                    Yawer.update(yaw_target);
                }
                else if(vision_aim_state == VISION_AIM)
                {
                    Yawer.update(_YawCorrectionAngle + Yaw_Angle[HitTarget]); // 更改Yaw轴角度
                    /*todo
                    测试时，暂时不管视觉
                    storage_base_angle = Yaw_Angle[HitTarget];

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
					Yawer.update(yaw_target);
					Yaw_Angle[HitTarget] = yaw_target;
                    */
                }
                else{
                    //意外情况
                    Yawer.disable();
                    //continue;
                }
            }
            else{
                //未完成校准
                //进行校准，校准完成后，自动改变校准标志
                Yawer.init();
            }
			//计算电机pid
			Yawer.adjust();
		}
        /*关控保护*/
        else
        {
            Yawer.disable();
        }
	    /*打包发送*/
        MotorMsgPack(Tx_Buff1, Yawer.YawMotor);
        xQueueSend(CAN1_TxPort, &Tx_Buff1.Id1ff, 0);
        //xQueueSend(CAN2_TxPort, &Tx_Buff1.Id1ff, 0);
	}
}