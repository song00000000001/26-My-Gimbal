#include "comm_protocal.h"

#include <stdarg.h>
#include <stdio.h>

// 发送装甲板控制包
// void SendFanPacket(uint8_t id,uint8_t cmd,light_color_enum color, uint8_t stage) {
//     CAN_COB CAN_TxMsg = {};
//     CAN_TxMsg.IdType = Can_STDID;
//     CAN_TxMsg.ID = CAN_SEND_ID_BASE + id; // 分控 ID 作为低字节
//     // CAN_TxMsg.DLC = 3;
//     // CAN_TxMsg.Data[2] = g_SystemState.SysMode;     // 当前系统模式
//     xQueueSend(CAN1_TxPort, &CAN_TxMsg, 0);
//     xQueueSend(CAN2_TxPort, &CAN_TxMsg, 0);
// }

// 分控反馈数据处理函数
// void FanFeedbackProcess(CAN_COB &CAN_RxMsg)
// {
//     // 1. 校验数据包
//     uint8_t sub_ctrl_id = CAN_RxMsg.ID - CAN_RECEIVE_ID_BASE;
//     if (sub_ctrl_id >= 1 && sub_ctrl_id <= 5 && CAN_RxMsg.DLC == 2) {
//         // //更新全局状态
//         // g_SystemState.CurrentHitID = sub_ctrl_id;
//         // g_SystemState.CurrentHitScores = CAN_RxMsg.Data[0];
//     }
// }

void my_printf(uint8_t port_id, const char* format, ...)
{  
    
    USART_COB TxMsg;

    va_list args;
    va_start(args, format);
    // 直接格式化到结构体的数组里
    int len = vsnprintf((char*)TxMsg.data, UART1_TX_BUFFER_SIZE, format, args);
    va_end(args);
    if (len > 0 && len < UART1_TX_BUFFER_SIZE)
    {        
        TxMsg.port_num = port_id; // 调试串口是 USART1
        TxMsg.len = len;
		BaseType_t result = xQueueSend(USART_TxPort, &TxMsg, 0); // 0 表示不等待
        if (result != pdPASS) {
			result=0;
        }
    }
    va_end(args);
}


bool send_motor_packet(uint8_t port_id,const uint8_t* data, uint8_t len)
{
    static USART_COB TxMsg;
    TxMsg.port_num = port_id; // 调试串口是 USART1
    TxMsg.len = len;
    memcpy(TxMsg.data, data, len);
    BaseType_t result = xQueueSend(USART_TxPort, &TxMsg, 0); // 0 表示不等待
    if (result != pdPASS) {
        result= 0; // 发送失败处理
    }
    return (bool)result;
}