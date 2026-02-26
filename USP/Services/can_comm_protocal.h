#pragma once

#include "SRML.h"
#include "global_data.h"
#include "robot_config.h"

#ifdef __cplusplus
extern "C" {
#endif



#ifdef __cplusplus
}
#endif

void SendFanPacket(uint8_t id,uint8_t cmd,light_color_enum color, uint8_t stage) ;
void FanFeedbackProcess(CAN_COB &CAN_RxMsg);
void my_printf(uint8_t port_num, const char* format, ...);
void hit_feedback_to_uart(uint8_t hitID,uint8_t scores);// 通过串口发送击打反馈，供上位机显示或调试使用
void small_enegy_settlement(uint8_t anverage_round);// 小能量机关结算函数，根据平均环数计算得分并发送反馈
void big_enegy_settlement(uint8_t average_round, uint8_t actived_arms);// 大能量机关结算函数，根据平均环数和激活灯臂数计算得分并发送反馈