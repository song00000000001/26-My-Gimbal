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

bool send_motor_packet(uint8_t port_num, uint8_t* data, uint16_t len);
void my_printf(uint8_t port_num, const char* format, ...);
