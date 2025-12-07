
#include "internal.h"
#include "global_data.h"
#include "robot_config.h"

/**
 * @brief Yaw轴控制任务
 * @parma None
 * @return None
 */



enum vision_aim_state_enum
{
    MANUAL_AIM = 0,
    VISION_AIM = 1,
    CORRECT_AIM = 2
};


void Yaw_Task(void *arg)
{
	Motor_CAN_COB Tx_Buff1;
	TickType_t xLastWakeTime_t;
    xLastWakeTime_t = xTaskGetTickCount();

    static bool yaw_control_state = 0;		//yaw轴控制状态
    static float yaw_target = 0;//, yaw_goal = 0, igniter_target_pos = 0, igniter_goal_pos = 0;
    static float yaw_correct_angle;        //yaw轴修正角
    static float default_yaw_target[2]; // 默认前哨站和基地角度

    test_servo_action(); // 测试舵机动作

	for (;;)
	{
		vTaskDelayUntil(&xLastWakeTime_t, 1);
		if (DR16.GetStatus() == DR16_ESTABLISHED && DR16.GetS2() != SW_UP)
		{
            if(DR16.GetS2() == SW_MID) // 右拨杆中档，手动模式
            {
                yaw_control_state = MANUAL_AIM;
            }
            else if(DR16.GetS2() == SW_DOWN) // 右拨杆朝下，视觉模式
            {
                yaw_control_state = CORRECT_AIM;
            }

            //如果已经校准过
            if (Yawer.is_Yaw_Init() == 1){
                if(yaw_control_state == MANUAL_AIM)
                {
                    //yaw_correct_angle += DR16.Get_RY_Norm() * 0.1f; // 手动微调
                    //yaw_correct_angle = std_lib::constrain(yaw_correct_angle, -10.2f, 10.2f);
                    yaw_target -= DR16.Get_LX_Norm() * 0.002f;
                    yaw_target = std_lib::constrain(yaw_target, -10.2f, 10.2f);
                    Yawer.update(yaw_target);
                }
                else if(yaw_control_state == VISION_AIM)
                {
                    /*todo
                    测试时，暂时不管视觉
                    storage_base_angle = default_yaw_target[HitTarget];

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
					default_yaw_target[HitTarget] = yaw_target;
                    */
                }
                else{
                     //固定修正值模式
                     Yawer.update(yaw_correct_angle + default_yaw_target[HitTarget]); // 更改Yaw轴角度
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
        xQueueSend(CAN2_TxPort, &Tx_Buff1.Id1ff, 0);
	}
}