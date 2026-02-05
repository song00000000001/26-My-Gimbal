#include "internal.h"
#include "global_data.h"
#include "motor_ctrl_driver.h"
#include "remote_ctrl_driver.h"
#include "robot_config.h"
#include "ws2812_ctrl_driver.h"

//辅助函数声明
void GenerateBETargets();
bool IsTarget(uint8_t id);
void RemoveTarget(uint8_t id);
void SendFanPacket(uint8_t id,uint8_t cmd,light_color_enum color, uint8_t stage);
void UpdateArmorsLight();
void LightArmors();
void ResetArmors();
void lightSuccessFlash(int8_t num);
void generateSETarget();
void updateSEArmorLight();
bool isSETarget(uint8_t id);
void R_light(light_color_enum color);
void FanFeedbackProcess(CAN_COB &CAN_RxMsg);

//状态机任务
void task_state_machine(void *arg)
{
    // 任务频率控制
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1);
    uint32_t main_task_now = xTaskGetTickCount();

    Motor_CAN_COB Tx_Buff = {};

    Debugger={
        .enable_debug_mode=0,
    };
     // --- 初始化部分 ---
    // 启动 ADC DMA (只需要启动一次)
    static uint32_t adcSeed = 0;
    adc_start(&adcSeed);
    
    // 稍微延时一下，确保 DMA 已经搬运了至少一次数据到 adcSeed
    // (因为 DMA 启动到完成第一次传输需要一点点时间，虽然极短)
    vTaskDelay(5); 

    // 设置随机数种子 (只需要执行一次)
    // 结合 ADC 悬空值和当前时间戳，保证每次上电的随机序列都不同
    srand(adcSeed + xTaskGetTickCount()); 

    g_SystemState.BE_Group = 0;
    g_SystemState.BE_State = BE_GENERATE_TARGET;
    for (;;)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        main_task_now = xTaskGetTickCount();

        Remote_Ctrl_Snapshot_Copy(&FS_I6X_Snap, &FS_I6X);
        
        // 处理遥控器连接状态及模式切换
        if (FS_I6X_Snap.Status != ESTABLISHED) {

        }
        else{
            if(FS_I6X_Snap.S1==SW_UP){
                
            }
            if(FS_I6X_Snap.S1==SW_DOWN){
                
            }
            if(FS_I6X_Snap.S1==SW_MID){
                
            }
        }
        
        if(g_SystemState.target_mode == 0) // 停止/待机
            g_SystemState.SysMode=idle;
        else if(g_SystemState.target_mode == 1) // 激活
            g_SystemState.SysMode=wait_start;

        // 模式判断
        if(g_SystemState.SysMode == idle) // 待机
        {
            g_SystemState.BE_Group = 0;// 重置轮数
            g_SystemState.BE_State = BE_GENERATE_TARGET;// 重置状态机
            g_SystemState.BE_Targets[0] = 0;
            g_SystemState.BE_Targets[1] = 0;
            g_SystemState.IsHit = 0;
            g_SystemState.CurrentHitID = 0;
            g_SystemState.SE_Group = 0; // 重置小能量轮数
            g_SystemState.SE_State = SE_GENERATE_TARGET; // 重置小能量状态机
            ResetArmors(); // 熄灭所有装甲板
            R_light(color_off);
            switch (g_SystemState.target_mode)
            {
            case 0: // 停止/待机
                g_SystemState.SysMode=idle;
                break;
            case 1: // 激活
            case 2: // 小能量机关
            case 3: // 大能量机关
            case 4: // 连续小能量机关
            case 5: // 连续大能量机关
                g_SystemState.SysMode=wait_start;
                break;
            default:
                break;
            }
        }
        else if(g_SystemState.SysMode == wait_start) // 等待开始
        {
            g_SystemState.BE_Group = 0;// 重置轮数
            g_SystemState.BE_State = BE_GENERATE_TARGET;// 重置状态机
            g_SystemState.BE_Targets[0] = 0;
            g_SystemState.BE_Targets[1] = 0;
            g_SystemState.IsHit = 0;
            g_SystemState.CurrentHitID = 0;
            g_SystemState.SE_Group = 0; // 重置小能量轮数
            g_SystemState.SE_State = SE_GENERATE_TARGET; // 重置小能量状态机
            ResetArmors(); // 熄灭所有装甲板
            if(g_SystemState.set_color == 0)
                R_light(color_red);
            else if(g_SystemState.set_color == 1)
                R_light(color_blue);

            vTaskDelay(1000);

            switch (g_SystemState.target_mode)
            {
            case 0: // 停止/待机
                g_SystemState.SysMode=idle;
                break;
            case 1: // 激活
                g_SystemState.SysMode=wait_start;
                break;
            case 2: // 小能量机关
            case 4: // 连续小能量机关
                g_SystemState.SysMode = small_energy; // 切换到小能量机关
                break;
            case 3: // 大能量机关
            case 5: // 连续大能量机关
                g_SystemState.SysMode = big_energy; // 切换到大能量机关
                break;
            default:
                break;
            }

        }
        else if(g_SystemState.SysMode == success) // 通关成功
        {
            // 全部点亮
            LightArmors();
            vTaskDelay(2000); 
            g_SystemState.target_mode = 0; // 结束，回到待机
        }
        else if(g_SystemState.SysMode == small_energy) 
        {
            uint32_t now = xTaskGetTickCount();

            // 状态机处理
            switch (g_SystemState.SE_State)
            {
            case SE_GENERATE_TARGET: // 生成目标
                generateSETarget();
                updateSEArmorLight();
                g_SystemState.SE_StateTimer = now;
                g_SystemState.SE_State = SE_WAIT_HIT; // 切换到等待击打
                break;
                
            case SE_WAIT_HIT: // 等待击打 (2.5s)
                // 超时失败
                if (now - g_SystemState.SE_StateTimer > 2500) {
                    ResetArmors(); // 熄灭所有装甲板
                    vTaskDelay(500);
                    g_SystemState.SE_Group = 0; // 重置轮数
                    if(g_SystemState.target_mode == 4)
                        g_SystemState.SE_State = SE_GENERATE_TARGET;
                    else
                        g_SystemState.target_mode = 1; // 结束，回到待机
                }
                
                // 击打判定
                if (g_SystemState.CurrentHitID != 0) {
                    uint8_t hitID = g_SystemState.CurrentHitID;
                    g_SystemState.CurrentHitID = 0;
                    
                    if (hitID == g_SystemState.SE_TargetID) {
                        // 击中目标，进入下一轮
                        g_SystemState.SE_Group++; // 轮数加一
                        if(g_SystemState.SE_Group >= 5) {
                            // 全部通关
                            if(g_SystemState.target_mode == 4){
                                lightSuccessFlash(2); // 闪烁提示成功
                                g_SystemState.SE_Group = 0; // 重置轮数
                                g_SystemState.SE_State = SE_GENERATE_TARGET;
                            }
                            else{
                                lightSuccessFlash(-4); // 闪烁提示成功
                                g_SystemState.target_mode = success; // 结束
                            }
                            break;
                        }
                        SendFanPacket(hitID, FAN_CMD_HIT, g_SystemState.TargetColor, 0);
                        vTaskDelay(200);
                        g_SystemState.SE_State = SE_GENERATE_TARGET; 
                    } 
                    else {
                        // 打错，重置
                        ResetArmors();
                        vTaskDelay(500);
                        g_SystemState.SE_Group = 0; // 重置轮数
                        if(g_SystemState.target_mode == 4)
                            g_SystemState.SE_State = SE_GENERATE_TARGET;
                        else
                            g_SystemState.target_mode = 1; // 结束，回到待机
                    }
                }
                break;

            default:
                g_SystemState.SE_State = SE_GENERATE_TARGET;
                break;
            }

            switch (g_SystemState.target_mode)
            {
            case 0: // 停止/待机
                g_SystemState.SysMode=idle;
                break;
            case 1: // 激活
            case 3: // 大能量机关
            case 5: // 连续大能量机关
                g_SystemState.SysMode=wait_start;
                break;
            case 2: // 小能量机关
            case 4: // 连续小能量机关
                g_SystemState.SysMode = small_energy; // 切换到小能量机关
                break;
            default:
                break;
            }
        }
        else if(g_SystemState.SysMode == big_energy) // 大能量机关
        {     
            uint32_t now = xTaskGetTickCount();

            // 状态机处理
            switch (g_SystemState.BE_State)
            {
            case BE_GENERATE_TARGET: // GENERATE_TARGET
                if (g_SystemState.BE_Group >= 5) {
                    // 全部通关
                    lightSuccessFlash(-4); // 闪烁提示成功
                    g_SystemState.SysMode = success; // 结束
                } 
                else {
                    GenerateBETargets();
                    UpdateArmorsLight();
                    g_SystemState.BE_StateTimer = now;
                    g_SystemState.BE_State = BE_WAIT_HIT_1; // 切换到 WAIT_HIT_1
                }
                break;
                
            case BE_WAIT_HIT_1: // WAIT_HIT_1 (第一阶段判定：2.5s)
                // 超时失败
                if (now - g_SystemState.BE_StateTimer > 2500) {
                    g_SystemState.BE_Group = 0; // 重置轮数
                    if(g_SystemState.target_mode == 5){
                        g_SystemState.BE_State = BE_GENERATE_TARGET;
                    }
                    else{
                        g_SystemState.target_mode = 1; // 结束，回到待机
                    }
                }
                
                // 击打判定
                if (g_SystemState.CurrentHitID != 0) {
                    uint8_t hitID = g_SystemState.CurrentHitID;
                    g_SystemState.CurrentHitID = 0;
                    
                    if (IsTarget(hitID)) {
                        // 击中其中一个，进入连击窗口
                        SendFanPacket(hitID, FAN_CMD_HIT, g_SystemState.TargetColor, g_SystemState.BE_Group + 1);
                        RemoveTarget(hitID); // 剩下的是要打的
                        
                        // 进入 Stage 2，重置计时器
                        g_SystemState.BE_StateTimer = now;
                        g_SystemState.BE_State = BE_WAIT_HIT_2; 
                    } else {
                        // 打错
                        g_SystemState.BE_Group = 0; // 重置轮数
                        if(g_SystemState.target_mode == 5){
                            g_SystemState.BE_State = BE_GENERATE_TARGET;
                        }
                        else{
                            g_SystemState.target_mode = 1; // 结束，回到待机
                        }

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
                    
                    if (IsTarget(hitID)) {
                        // 击中剩下那个 -> 双杀成功
                        SendFanPacket(hitID, FAN_CMD_HIT, g_SystemState.TargetColor, g_SystemState.BE_Group + 1);
                        vTaskDelay(200);
                        
                        g_SystemState.BE_Group++;
                        g_SystemState.BE_State = BE_GENERATE_TARGET;
                    } else {
                        // 连击阶段打错，也判负
                        g_SystemState.BE_Group = 0; // 重置轮数
                        if(g_SystemState.target_mode == 5){
                            g_SystemState.BE_State = BE_GENERATE_TARGET;
                        }
                        else{
                            g_SystemState.target_mode = 1; // 结束，回到待机
                        }
                    }
                }
                break;
                
            default:
                g_SystemState.BE_State = BE_GENERATE_TARGET;
                break;
            }
        }
                
        //记录任务剩余栈空间
        #ifdef INCLUDE_uxTaskGetStackHighWaterMark
        //Stack_Remain.task_state_machine_stack_remain = uxTaskGetStackHighWaterMark(NULL);
        #endif
    }
}



// --- 大能量机关辅助函数 ---
// 生成两个不重复的随机目标(1-5)
void GenerateBETargets() {
    g_SystemState.BE_Targets[0] = (rand() % 5) + 1;
    do {
        g_SystemState.BE_Targets[1] = (rand() % 5) + 1;
    } while (g_SystemState.BE_Targets[1] == g_SystemState.BE_Targets[0]);
}

// 检查ID是否为当前目标
bool IsTarget(uint8_t id) {
    return (id == g_SystemState.BE_Targets[0] || id == g_SystemState.BE_Targets[1]);
}

// 移除已击打的目标(用于连击窗口判定)
void RemoveTarget(uint8_t id) {
    if (g_SystemState.BE_Targets[0] == id) g_SystemState.BE_Targets[0] = 0;
    if (g_SystemState.BE_Targets[1] == id) g_SystemState.BE_Targets[1] = 0;
}

// 发送装甲板控制包
void SendFanPacket(uint8_t id,uint8_t cmd,light_color_enum color, uint8_t stage) {
    CAN_COB CAN_TxMsg = {};
    CAN_TxMsg.IdType = Can_STDID;
    CAN_TxMsg.ID = CAN_SEND_ID_BASE + id; // 分控 ID 作为低字节
    CAN_TxMsg.DLC = 3;
    if(cmd==FAN_CMD_RESET){
        CAN_TxMsg.Data[0] = static_cast<uint8_t>(color_off); // 熄灭
        CAN_TxMsg.Data[1] = 0;     // 阶段无效
       
    }
    else if(cmd==FAN_CMD_SELECT){
        CAN_TxMsg.Data[0] = static_cast<uint8_t>(color); // 颜色
        CAN_TxMsg.Data[1] = stage;     // 阶段
    }
    else if(cmd==FAN_CMD_HIT){
        if(color==color_red)
            CAN_TxMsg.Data[0] = static_cast<uint8_t>(color_hit_red); // 击打红色
        else if(color==color_blue)
            CAN_TxMsg.Data[0] = static_cast<uint8_t>(color_hit_blue); // 击打蓝色
        CAN_TxMsg.Data[1] = stage;     // 阶段
    }
    CAN_TxMsg.Data[2] = g_SystemState.SysMode;     // 当前系统模式
    xQueueSend(CAN1_TxPort, &CAN_TxMsg, 0);
    xQueueSend(CAN2_TxPort, &CAN_TxMsg, 0);
}

// 更新所有装甲板灯光状态
void UpdateArmorsLight() {
    uint8_t stage = g_SystemState.BE_Group + 1; // 阶段1-5
    for (int i = 1; i <= 5; i++) {
        if (IsTarget(i)) {
            // 是目标：亮起瞄准灯 (NORMAL)
            SendFanPacket(i,FAN_CMD_SELECT,g_SystemState.TargetColor, stage);
        } else {
            // 非目标：熄灭/背景灯 (RESET，分控根据stage显示进度条)
            SendFanPacket(i,FAN_CMD_RESET,color_off, stage);
        }
    }
}

void LightArmors() {
    for (int i = 1; i <= 5; i++) {
        // 全部点亮
        SendFanPacket(i,FAN_CMD_HIT,g_SystemState.TargetColor, g_SystemState.BE_Group);
    }
}

void ResetArmors() {
    for (int i = 1; i <= 5; i++) {
        // 全部熄灭
        SendFanPacket(i,FAN_CMD_RESET,color_off, g_SystemState.BE_Group);
    }
}

// 成功后闪烁重置,num>0正闪亮灭,num<0反闪灭亮
void lightSuccessFlash(int8_t num) {
    if(num < 0) {
        num = -num;
        for (uint8_t flash = 0; flash < num; flash++) {
            // 全部熄灭
            ResetArmors();
            vTaskDelay(500);
            // 全部点亮
            LightArmors();
            vTaskDelay(500);
        }
    }
    else if(num > 0) {
        for (uint8_t flash = 0; flash < num; flash++) {
            // 全部点亮
            LightArmors();
            vTaskDelay(500);
            // 全部熄灭
            ResetArmors();
            vTaskDelay(500);
        }
    }
    else 
        return;
}   


// 小能量机关辅助函数
// 小能量机关比较简单,只是在5个装甲板中随机选择一个点亮,然后在2.5秒内等待击打
// 如果击中则继续下一个,如果超时未击中或者打错则重置
// 1. 生成随机目标
void generateSETarget() {
    g_SystemState.SE_TargetID = (rand() % 5) + 1;
}
// 2. 更新装甲板灯光状态
void updateSEArmorLight() {
    for (int i = 1; i <= 5; i++) {
        if (i == g_SystemState.SE_TargetID) {
            // 是目标：亮起瞄准灯
            SendFanPacket(i, FAN_CMD_SELECT, g_SystemState.TargetColor, 0);
        } else {
            // 非目标：熄灭
            SendFanPacket(i, FAN_CMD_RESET, color_off, 0);
        }
    }
}
// 4. 是否为目标
bool isSETarget(uint8_t id) {
    return (id == g_SystemState.SE_TargetID);
}


void R_light(light_color_enum color){
    switch(color){
        case color_red:
        case color_hit_red:
            ws2312_show(255, 0, 0); // 红色
            break;
        case color_blue:
        case color_hit_blue:
            ws2312_show(0, 0, 255); // 蓝色
            break;
        case color_off:
        default:
            ws2312_show(0, 0, 0); // 关灯
            break;
    }
}

//分控反馈数据处理函数
void FanFeedbackProcess(CAN_COB &CAN_RxMsg)
{
    // 1. 校验数据包
    uint8_t sub_ctrl_id = CAN_RxMsg.ID - CAN_RECEIVE_ID_BASE;
    if (sub_ctrl_id >= 1 && sub_ctrl_id <= 5 && CAN_RxMsg.DLC == 2) {
        uint16_t hit_Round = (CAN_RxMsg.Data[0] << 8) | CAN_RxMsg.Data[1];
        //更新全局状态
        g_SystemState.CurrentHitID = sub_ctrl_id;
        if(isSETarget(sub_ctrl_id)){
            // 小能量机关击中目标
            g_SystemState.IsHit = 1;
            g_SystemState.CurrentHitRound+=hit_Round;
        }
        else if(IsTarget(sub_ctrl_id)){
            // 大能量机关击中目标
            RemoveTarget(sub_ctrl_id);
            g_SystemState.IsHit = 1;
        }
        else{
            // 击中非目标
            g_SystemState.IsHit = 0;
        }
    }
}