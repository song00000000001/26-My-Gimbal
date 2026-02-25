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
void my_printf(const char* format, ...);