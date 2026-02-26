#include "global_data.h"
#include "can_comm_protocal.h"


// --- 大能量机关辅助函数 ---
// 生成两个不重复的随机目标(1-5)
void GenerateBETargets() {
    g_SystemState.BE_Targets[0] = (rand() % 5) + 1;
    do {
        g_SystemState.BE_Targets[1] = (rand() % 5) + 1;
    } while (g_SystemState.BE_Targets[1] == g_SystemState.BE_Targets[0]);
}

// 移除已击打的目标(用于连击窗口判定)
void RemoveTarget(uint8_t id) {
    if (g_SystemState.BE_Targets[0] == id) g_SystemState.BE_Targets[0] = 0;
    if (g_SystemState.BE_Targets[1] == id) g_SystemState.BE_Targets[1] = 0;
}

// 更新所有装甲板灯光状态
void updateBEArmorLight() {
    uint8_t stage = g_SystemState.BE_Group + 1; // 阶段1-5
    for (int i = 1; i <= 5; i++) {
        if (i == g_SystemState.BE_Targets[0] || i == g_SystemState.BE_Targets[1]) {
            // 是目标：亮起瞄准灯 (NORMAL)
            SendFanPacket(i,FAN_CMD_SELECT,g_TargetCtrl.TargetColor, stage);
        } else {
            // 非目标：熄灭/背景灯 (RESET，分控根据stage显示进度条)
            SendFanPacket(i,FAN_CMD_RESET,color_off, stage);
        }
    }
}

void BE_reset() {
    g_SystemState.BE_Group = 0;
    g_SystemState.BE_State = BE_GENERATE_TARGET;
    g_SystemState.BE_Targets[0] = 0;
    g_SystemState.BE_Targets[1] = 0;
    g_SystemState.CurrentHitID = 0;
    g_SystemState.CurrentHitScores = 0;
    g_SystemState.BE_ActivedArms = 0;
    g_SystemState.BE_Scores = 0;
    ResetArmors(); // 熄灭所有装甲板
    my_printf(upper_uart_id, "BE reset\n");
}

void big_energy_logic() {

    uint32_t now = xTaskGetTickCount();

    // 状态机处理
    switch (g_SystemState.BE_State)
    {
    case BE_GENERATE_TARGET: // GENERATE_TARGET
        if (g_SystemState.BE_Group >= 5) {
            // 全部通关
            big_enegy_settlement(g_SystemState.BE_Scores, g_SystemState.BE_ActivedArms); // 结算，平均环数=轮数，激活灯臂数=2
            if(g_TargetCtrl.target_mode == tar_big_energy_continue){
                BE_reset();
            }
            else{
                lightSuccessFlash(-4); // 闪烁提示成功
                g_SystemState.SysMode = success; // 结束
            }
        } 
        else {
            GenerateBETargets();
            updateBEArmorLight();
            g_SystemState.BE_StateTimer = now;
            g_SystemState.BE_State = BE_WAIT_HIT_1; // 切换到 WAIT_HIT_1
        }
        break;
        
    case BE_WAIT_HIT_1: // WAIT_HIT_1 (第一阶段判定：2.5s)
        // 超时失败
        if (now - g_SystemState.BE_StateTimer > 2500) {
            BE_reset();
        }
        
        // 击打判定
        if (g_SystemState.CurrentHitID != 0) {
            uint8_t hitID = g_SystemState.CurrentHitID;
            g_SystemState.CurrentHitID = 0;
            g_SystemState.BE_Scores+= g_SystemState.CurrentHitScores;
            g_SystemState.CurrentHitScores = 0;
            g_SystemState.BE_ActivedArms++; // 激活灯臂数加一
            hit_feedback_to_uart(hitID, g_SystemState.CurrentHitScores); 
            if (g_SystemState.BE_Targets[0] == hitID || g_SystemState.BE_Targets[1] == hitID) {
                // 击中其中一个，进入连击窗口
                SendFanPacket(hitID, FAN_CMD_HIT, g_TargetCtrl.TargetColor, g_SystemState.BE_Group + 1);
                RemoveTarget(hitID); // 剩下的是要打的
                
                // 进入 Stage 2，重置计时器
                g_SystemState.BE_StateTimer = now;
                g_SystemState.BE_State = BE_WAIT_HIT_2; 
            } 
            else {
                // 打错
                BE_reset();
            }
        }
        break;

    case BE_WAIT_HIT_2: // WAIT_HIT_2 (第二阶段连击：1s)
        // 超时结束 -> 成功（单杀）
        if (now - g_SystemState.BE_StateTimer > 1000) {
            g_SystemState.BE_Group++; // 晋级
            g_SystemState.BE_State = BE_GENERATE_TARGET; 
        }
        
        // 击打判定
        if (g_SystemState.CurrentHitID != 0) {
            uint8_t hitID = g_SystemState.CurrentHitID;
            g_SystemState.CurrentHitID = 0;
            g_SystemState.BE_Scores+= g_SystemState.CurrentHitScores;
            g_SystemState.CurrentHitScores = 0;
            g_SystemState.BE_ActivedArms++; // 激活灯臂数加一
            hit_feedback_to_uart(hitID, g_SystemState.CurrentHitScores); 
            if (g_SystemState.BE_Targets[0] == hitID || g_SystemState.BE_Targets[1] == hitID) {
                // 击中剩下那个 -> 双杀成功
                SendFanPacket(hitID, FAN_CMD_HIT, g_TargetCtrl.TargetColor, g_SystemState.BE_Group + 1);
                vTaskDelay(20);
                g_SystemState.BE_Group++;
                g_SystemState.BE_State = BE_GENERATE_TARGET;
            } 
            else {
                // 连击阶段打错，也判负
                BE_reset();
            }
        }
        break;
        
    default:
        BE_reset();
        break;
    }
}