#include "internal.h"
#include "global_data.h"
#include "robot_config.h"


/*todo
    song
    考虑做软启动和软停止。
    考虑双电机切换运行，一个主，一个备份。
    如果主电机温度过高或者转速异常，则切换到备份电机运行，等主电机温度下降后再切换回来。
    目前先不做，等有需求再说。
*/

void task_motor_ctrl(void *arg)
{
    Motor_CAN_COB Tx_Buff = {};

    TickType_t xLastWakeTime_t;
    xLastWakeTime_t = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(2);

    motor_ctrl.set_motor_mode(MODE_SPEED);
    motor_ctrl.mymotor_pid_spd.SetPIDParam(0.001,0,0,0,10);

    for (;;)
    {
        vTaskDelayUntil(&xLastWakeTime_t, xFrequency);

        // 实时更新时间戳
        uint32_t time_clock = xTaskGetTickCount();

        // 状态机处理
        switch(g_SystemState.SysMode)
        {
            case wait_start: // 等待开始
                g_SystemState.TargetSpeed = 10 * 71 * 4 * 1.0f; // 恒定速度: 10 rpm * 减速比
                break;
            case small_energy: // 小能量机关
                // 恒定速度: 10 rpm * 减速比... (原公式保留)
                // 假设 receivedata3.speed 原本是归一化系数或转速, 这里直接用 debug 参数
                g_SystemState.TargetSpeed = 10 * 71 * 4 * g_SystemState.SmallEnergy_Speed;
                break;
                
            case big_energy: // 大能量机关
                {
                    float param_a = g_SystemState.BigEnergy_A;
                    float param_w = g_SystemState.BigEnergy_W;
                    
                    // 参数限幅 (保持原有逻辑)
                    if(param_a > 1.045f) param_a = 1.045f;
                    else if(param_a < 0.780f) param_a = 0.780f;
                    
                    if(param_w > 2.000f) param_w = 2.000f;
                    else if(param_w < 1.884f) param_w = 1.884f;

                    // 计算大符速度
                    g_SystemState.TargetSpeed = (param_a * sin(param_w * time_clock/1000.0f) + 2.090f - param_a) * 71 * 4 * 60 / 6.28f;
                    
                }
                break;

            case success: // 通关成功
                g_SystemState.TargetSpeed = 0;
                break;

            case idle: // 停止/待机                
            default:
                g_SystemState.TargetSpeed = 0;
                g_SystemState.BE_Group = 0;
                g_SystemState.BE_State = BE_GENERATE_TARGET;
                g_SystemState.BE_Targets[0] = 0;
                g_SystemState.BE_Targets[1] = 0;
                g_SystemState.IsHit = 0;
                g_SystemState.CurrentHitID = 0;
                g_SystemState.TargetColor = color_off;
                break;
        }  
        // 速度调整
        //target设置
        //motor_ctrl.set_motor_target_speed(g_SystemState.TargetSpeed);
        //current获取
        g_SystemState.RealSpeed = motor_ctrl.get_motor_speed();
        //pid计算
        motor_ctrl.adjust();
        if(g_SystemState.SysMode == idle)
        {
            motor_ctrl.set_motor_mode(MODE_ERROR); // 失能
            motor_ctrl.motor_output(false);           //不输出
        }
        else{
            motor_ctrl.set_motor_mode(MODE_SPEED); // 速度环
            motor_ctrl.motor_output(true);            //输出设置
        }

        // can发送速度指令
        MotorMsgPack(Tx_Buff, motor_ctrl.mymotor);
        // 强制将 ID 改为 0x3FE,后续考虑优化成配置项
        Tx_Buff.Id200.ID = 0x3FE; 
        // 发送给电机
        xQueueSend(CAN1_TxPort, &Tx_Buff.Id200, 0);
        xQueueSend(CAN2_TxPort, &Tx_Buff.Id200, 0);

        #ifdef INCLUDE_uxTaskGetStackHighWaterMark
        StackWaterMark_Get(motor_ctrl);
        #endif
    }
}
