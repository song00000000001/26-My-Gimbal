#include "can_comm_protocal.h"

#include <stdarg.h>
#include <stdio.h>

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

//分控反馈数据处理函数
void FanFeedbackProcess(CAN_COB &CAN_RxMsg)
{
    // 1. 校验数据包
    uint8_t sub_ctrl_id = CAN_RxMsg.ID - CAN_RECEIVE_ID_BASE;
    if (sub_ctrl_id >= 1 && sub_ctrl_id <= 5 && CAN_RxMsg.DLC == 2) {
        //更新全局状态
        g_SystemState.CurrentHitID = sub_ctrl_id;
        g_SystemState.CurrentHitScores = CAN_RxMsg.Data[0];
    }
}
/*
void my_printf(const char *format, ...)
{
    char buf[BLE_TX_BUF_LEN];
    va_list args;
    va_start(args, format);

    int len = vsnprintf(buf, BLE_TX_BUF_LEN, format, args);
    if (len > 0 && len < BLE_TX_BUF_LEN)
    {
      ble_print((uint8_t *)buf,len);
    }

    va_end(args);
}
*/
USART_COB TxMsg = {};
char TxBuf[UART1_RX_BUFFER_SIZE];

void my_printf(uint8_t port_num, const char* format, ...)
{  
    va_list args;
    va_start(args, format);
    int len = vsnprintf(TxBuf, UART1_RX_BUFFER_SIZE, format, args);
    if (len > 0 && len < UART1_RX_BUFFER_SIZE)
    {        
        TxMsg.port_num = port_num; // 调试串口是 USART1
        TxMsg.len = len;
        TxMsg.address = (uint8_t *)TxBuf;
        xQueueSend(USART_TxPort, &TxMsg, 0);
    }
    va_end(args);
    vTaskDelay(5); // 稍微延时,不然串口打印会乱
}

void hit_feedback_to_uart(uint8_t hitID,uint8_t scores){
    my_printf(upper_uart_id, "Hit ID: %d, Scores: %d\r\n", hitID, scores);
}

/*
一方机器人成功激活小能量机关后，该方所有机器人、前 哨站、基地均获得25%的防御增益，持续45秒。
小能量机关增益持续期间内，所有英雄、步兵、空 中机器人在获得经验时，额外获得原经验100%的经验，一方在一次小能量机关增益期间内通过此方 式最多共获得 1200 点额外经验。
*/
void small_enegy_settlement(uint8_t average_round){
    vTaskDelay(50);
    my_printf(upper_uart_id, "SE settlement,total Round: %d\r\n", average_round);
    vTaskDelay(10);
}

/*
## 大能量机关
- 大能量机关的每块装甲模块被划分为1~10环。一方大能量机关被激活后，系统将根据其击中的平均环数和激活灯臂数、为该方所有机器人提供时间不等、效果不同的攻击、防御和热量冷却增益，为该方前哨站、基地提供相应的防御增益，详见“表 5-17 平均环数与对应增益”。同时，大能量机关被激活时，将有750点经验平均分给激活方所有存活的英 雄、步兵、空中机器人。
表 5-17 平均环数与对应增益

| 平均环数区间 | 攻击增益 | 防御增益 | 热量冷却增益 |
| :----: | :--: | :--: | :----: |
| [1,3]  | 150% | 25%  |   无    |
| (3,7]  | 150% | 25%  |   2倍   |
| (7,8]  | 200% | 25%  |   2倍   |
| (8,9]  | 200% | 25%  |   3倍   |
| (9,10] | 300% | 50%  |   5倍   |
表 5-18 激活灯臂数与对应增益持续时间

| 激活灯臂数 | 增益持续时间（秒） |
| :---: | :-------: |
|   5   |    30     |
|   6   |    35     |
|   7   |    40     |
|   8   |    45     |
|   9   |    50     |
|  10   |    60     |
*/
void big_enegy_settlement(uint8_t average_round, uint8_t actived_arms){
    vTaskDelay(50);
    my_printf(upper_uart_id, "BE Settlement,Average Round: %d, Actived Arms: %d\r\n", average_round, actived_arms);
    vTaskDelay(50);
	if(average_round >= 0 && average_round <= 3){
        my_printf(upper_uart_id, "Attack Gain: 150%%, Defense Gain: 25%%, Heat Cooling Gain: None\r\n");
    }
    else if(average_round > 3 && average_round <= 7){
        my_printf(upper_uart_id, "Attack Gain: 150%%, Defense Gain: 25%%, Heat Cooling Gain: 2x\r\n");
    }
    else if(average_round > 7 && average_round <= 8){
        my_printf(upper_uart_id, "Attack Gain: 200%%, Defense Gain: 25%%, Heat Cooling Gain: 2x\r\n");
    }
    else if(average_round > 8 && average_round <= 9){
        my_printf(upper_uart_id, "Attack Gain: 200%%, Defense Gain: 25%%, Heat Cooling Gain: 3x\r\n");
    }
    else if(average_round > 9 && average_round <= 10){
        my_printf(upper_uart_id, "Attack Gain: 300%%, Defense Gain: 50%%, Heat Cooling Gain: 5x\r\n");
    }
    vTaskDelay(50);
    if(actived_arms >= 5 && actived_arms <= 10){
        uint8_t duration = 30 + (actived_arms-5) * 5; // 根据激活灯臂数计算增益持续时间
        if(actived_arms >= 10) duration = 60; //十组特殊处理
        my_printf(upper_uart_id, "Gain Duration: %d seconds\r\n", duration);
    }
    vTaskDelay(50);
}