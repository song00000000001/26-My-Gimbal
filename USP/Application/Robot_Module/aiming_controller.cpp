
#include "internal.h"
#include "global_data.h"
#include "robot_config.h"


void yaw_state_machine(){
    
    static float yaw_target = 0;//, yaw_goal = 0, igniter_target_pos = 0, igniter_goal_pos = 0;
    static float yaw_correct_angle;        //yaw轴修正角
    static float default_yaw_target[2]; // 默认前哨站和基地角度

    switch (Robot.Status.yaw_control_state)
    {
    case MANUAL_AIM:
        yaw_target -= DR16.Get_LX_Norm() * 0.002f;
        yaw_target = std_lib::constrain(yaw_target, -10.2f, 10.2f);
        Yawer.update(yaw_target);
        break;
    case CORRECT_AIM:
        //根据目标选择修正角度
        //固定修正值模式
        Yawer.update(yaw_correct_angle + default_yaw_target[HitTarget]); // 更改Yaw轴角度
        break;
    case VISION_AIM:
        //视觉模式
        //todo
        {/*todo
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
        //计算电机pid
        Yawer.adjust();
        */
       }
        break;
    case YAW_CALIBRATING:
        //校准模式
        //进行校准，校准完成后，自动改变校准标志
        Yawer.calibration();
        if(Yawer.is_Yaw_Init()){
            Robot.Status.yaw_control_state = MANUAL_AIM; //校准完成后，进入手动模式
        }
        break;
    default:
        Yawer.disable();
        break;
    }

}



