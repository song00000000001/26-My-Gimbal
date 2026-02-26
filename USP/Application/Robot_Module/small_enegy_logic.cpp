#include "global_data.h"
#include "can_comm_protocal.h"

// 小能量机关辅助函数
// 小能量机关比较简单,只是在5个装甲板中随机选择一个点亮,然后在2.5秒内等待击打,如果超时未击中或者打错则重置
// 如果击中则亮起该arm,然后再剩下的中随机选一个,直到完成5轮
// 1. 生成随机目标
void generateSETarget() {
    //参考洗牌算法,生成不重复的1~5的随机序列
    static uint8_t ids[5] = {1, 2, 3, 4, 5};
    for (int i = 4; i > 0; i--) {
        int j = rand() % (i + 1);
        uint8_t temp = ids[i];
        ids[i] = ids[j];
        ids[j] = temp;
    }
    for(int i = 0; i < 5; i++){
        g_SystemState.SE_TargetID_GROUP[i] = ids[i];
    }
}
// 2. 更新装甲板灯光状态
void updateSEArmorLight() {
    for (int i = 1; i <= 5; i++) {
        if (i == g_SystemState.SE_TargetID) {
            // 是目标：亮起瞄准灯
            SendFanPacket(i, FAN_CMD_SELECT, g_TargetCtrl.TargetColor, 0);
        } 
        else {
            // 非目标：熄灭
            SendFanPacket(i, FAN_CMD_RESET, color_off, 0);
        }
    }
}

void small_energy_logic() {

    uint32_t now = xTaskGetTickCount();

    // 状态机处理
    switch (g_SystemState.SE_State)
    {
    case SE_GENERATE_TARGET: // 生成目标
        g_SystemState.SE_Group = 0; // 重置轮数
        generateSETarget();
        g_SystemState.SE_StateTimer = now;
        g_SystemState.SE_State = SE_WAIT_HIT; // 切换到等待击打
        break;
        
    case SE_WAIT_HIT: // 等待击打 (2.5s)
        // 超时失败
        if (now - g_SystemState.SE_StateTimer > 2500) {
            ResetArmors(); // 熄灭所有装甲板
            vTaskDelay(500);
            g_SystemState.SE_Group = 0; // 重置轮数
            g_SystemState.SE_State = SE_GENERATE_TARGET;
        }
        
        g_SystemState.SE_TargetID=g_SystemState.SE_TargetID_GROUP[g_SystemState.SE_Group];
        // 击打判定
        if (g_SystemState.CurrentHitID != 0) {
            uint8_t hitID = g_SystemState.CurrentHitID;
            g_SystemState.CurrentHitID = 0;
            g_SystemState.SE_Scores += g_SystemState.CurrentHitScores;
            g_SystemState.CurrentHitScores = 0;

            hit_feedback_to_uart(hitID); // 通过串口发送击打反馈，供上位机显示或调试使用
            if (hitID == g_SystemState.SE_TargetID) {
                // 击中目标，进入下一轮
                g_SystemState.SE_Group++; // 轮数加一
                if(g_SystemState.SE_Group > 4) {
                    small_enegy_settlement(g_SystemState.SE_Scores); // 结算，传入得分
                    g_SystemState.SE_Scores = 0;
                    // 全部通关
                    if(g_TargetCtrl.target_mode == tar_small_energy_continue){
                        g_SystemState.SE_State = SE_GENERATE_TARGET;
                    }
                    else{
                        lightSuccessFlash(-4); // 闪烁提示成功
                        g_SystemState.SysMode = success; // 结束
                    }
                    break;
                }
                SendFanPacket(hitID, FAN_CMD_HIT, g_TargetCtrl.TargetColor, 0);
                vTaskDelay(20);

            } 
            else {
                // 打错，重置
                ResetArmors();
                vTaskDelay(100);
                g_SystemState.SE_State = SE_GENERATE_TARGET;
                break;
            }
   
            updateSEArmorLight();
            g_SystemState.SE_StateTimer = now;
        }
        break;

    default:
        g_SystemState.SE_State = SE_GENERATE_TARGET;
        break;
    }
}
