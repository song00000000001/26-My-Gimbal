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
        uint16_t hit_Round = (CAN_RxMsg.Data[0] << 8) | CAN_RxMsg.Data[1];
        //更新全局状态
        g_SystemState.CurrentHitID = sub_ctrl_id;
    }
}


void my_printf(const char* format, ...)
{
    USART_COB TxMsg = {};
    va_list args;
    va_start(args, format);
    int len = vsnprintf((char*)TxMsg.address, UART1_RX_BUFFER_SIZE, format, args);
    if (len > 0 && len < UART1_RX_BUFFER_SIZE)
    {        
        TxMsg.port_num = 1; // 调试串口是 USART1
        TxMsg.len = len;
        xQueueSend(USART_TxPort, &TxMsg, 0);
    }
    va_end(args);
    
}