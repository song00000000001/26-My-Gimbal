#include "internal.h"
#include "global_data.h"

/**
 * @brief 发射主控任务
 */

void LaunchCtrl(void *arg)
{
    Motor_CAN_COB Tx_Buff;

    // 使用 xTaskDelayUntil 的变量
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(1); // 1ms 周期
    xLastWakeTime = xTaskGetTickCount();

    // 非阻塞延时用的计时变量
    uint32_t state_timer_start = 0;
    
    // 目标切换防抖计时
    uint32_t target_switch_timer = 0;
    bool target_switching = false;

    // 本地数据快照
    DR16_Snapshot_t rc_data;

    for (;;)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        /* 数据快照 (Snapshot) - 解决数据一致性和锁性能问题 */
        // 假设 GetStatus 内部不加锁，或者如果需要锁，在这里加一次即可
        // xSemaphoreTake(DR16_mutex, portMAX_DELAY); 
        rc_data.Status = DR16.GetStatus();
        rc_data.S1 = DR16.GetS1();
        rc_data.S2 = DR16.GetS2();
        rc_data.RY_Norm = DR16.Get_RY_Norm();
        rc_data.RX_Norm = DR16.Get_RX_Norm();
        // xSemaphoreGive(DR16_mutex);

        if (rc_data.Status == DR16_ESTABLISHED)
        {
            if (rc_data.S1 != SW_DOWN || vision_aim_state == 1) // 视觉调整yaw时不允许发射
            {
                switch (state)
                {
                case DEINIT: // 初始化
                    Launch.Igniter_On();    // 解锁扳机
                    Launch.Deliver_Init(); // 左右滑块初始化
                    Launch.Igniter_Init(); // 扳机丝杆
                    if (Launch.Igniter_Init_flag == 1 && Launch.is_Deliver_Init() == 1)
                    {
                        state = WAIT_ACT;
                    }
                    break;
                case WAIT_ACT: // 等待蓄力指令,并且丝杆移动到指定位置
                    Launch.Deliver_Init();
                    Launch.Igniter_On();
                    if (cnt == 0)
                    {
                        status++;
                        cnt = 1; // 使status只加一次
                    }
                    if (rc_data.S1 == SW_MID || rc_data.S1 == SW_DOWN) // 丝杆数据用调参板数据
                    {
                        igniter_goal_pos = DartsData[status - 1].Ignitergoal[HitTarget];
                        igniter_goal_pos = std_lib::constrain(igniter_goal_pos, 3.f, 260.f);
                        Launch.Igniter_Pos_Set(igniter_target_pos);
                    }
                    if (rc_data.S2 == SW_DOWN && Launch.PID_Igniter_Angle.Error < 0.1) // 右边拨杆下拨且丝杆移动到位后，滑块拉下，扳机扣下
                    {
                        state = PULL;
                    }
                    break;

                case PULL:
                    if (Launch.Pull_Ready_flag != 1)
                    {
                        Launch.Igniter_Off();
                        Launch.Deliver_Pull();
                    }
                    else
                    {
                        Launch.Igniter_Off(); // 扳机扣下
                        vTaskDelay(300);
                        state = BACK;
                    }
                    break;
                case BACK:
                    Launch.Igniter_Off();
                    Launch.Deliver_Init();
                    open = 1; // 小风车强制切换到发射状态
                    goal = 2750;
                    if (Launch.is_Deliver_Init() == 1)
                    {
                        state = WAIT_SHOOT;
                    }
                    break;
                case WAIT_SHOOT: // 进入待发射状态
                    Launch.Deliver_Init();
                    if (rc_data.S2 == SW_UP) // 扳机释放，发射！
                    {
                        Launch.Igniter_On();
                        state = WAIT_ACT; // 回到等待指令状态，准备下一次发射
                        vTaskDelay(2000);
                        open = 0;
                        cnt = 0; // 更新装填状态，准备装填下一发
                    }
                    else
                    {
                        Launch.Igniter_Off();
                    }
                    break;

                default:
                    break;
                }
            }
        }
        /*关控保护*/
        else
        {
            state = DEINIT; // 连接断开，回到初始化状态
            Launch.disable();
            Yaw.disable();
            loadermotor[0].Out = 0;
            /*todo
            song
                这里需要检查pid系统是否全部关闭，
                积分量是否全部清空。
                输出是否全部清零
                扳机舵机是否松开
                夹镖舵机是否夹紧
            */
            
        }

        /*丝杆电机进入调试模式，遥控可控制丝杆的前后*/
        if (state != PULL && rc_data.S1 == SW_UP && Launch.Igniter_Init_flag == 1) // 左拨杆朝上，进入调试模式
        {
            igniter_target_pos += rc_data.RY_Norm * 0.03f;
            igniter_target_pos = std_lib::constrain(igniter_target_pos, 0.f, 260.f);
            Launch.Igniter_Pos_Set(igniter_target_pos);
        }
        Launch.adjust();

        /*切换目标*/
       /* 6. 切换目标 (非阻塞优化) */
        if (rc_data.RY_Norm < -0.9f)
        {
            // 简单的状态机处理长按逻辑
            if (rc_data.RX_Norm < -0.9f || rc_data.RX_Norm > 0.9f) {
                if (!target_switching) {
                    target_switching = true;
                    target_switch_timer = xTaskGetTickCount();
                } 
                else {
                    if ((xTaskGetTickCount() - target_switch_timer) > 1000) {
                        // 确认切换
                        if (rc_data.RX_Norm < -0.9f) HitTarget = Outpost;
                        else HitTarget = Base;
                        // 这里不需要重置 target_switching，直到摇杆回中，防止重复触发
                    }
                }
            } 
            else {
                target_switching = false;
            }
        } 
        else {
            target_switching = false;
        }

        /*打包数据发送*/
        MotorMsgPack(Tx_Buff, Launch.DeliverMotor[L], Launch.DeliverMotor[R], Launch.IgniterMotor);
        xQueueSend(CAN2_TxPort, &Tx_Buff.Id200, 0);
        //这里说明同步带电机和行程电机都是can2总线，
        //后续进行can2接收测试时，可以和这里互相验证。
    }
}