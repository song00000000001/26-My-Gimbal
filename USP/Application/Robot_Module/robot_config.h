#pragma once
#include "SRML.h"
#include "tim.h"    
#include "adc.h"

#ifdef __cplusplus
extern "C" {
#endif

#define upper_uart_id 4 // 上位机通信串口
#define motor_uart_id 1 // 电机通信串口 

#define STACK_REMAIN_MONITER_ENABLE 0 // 是否启用任务栈剩余监测功能

// 帧头定义
#define FAN_PACKET_HEADER 0xA5
#define CAN_SEND_ID_BASE 0x210  // CAN 发送包头标识
#define CAN_RECEIVE_ID_BASE 0x220 // CAN 接收包头标识
#define CAN_FILTER_ID_MASK 0x7F0 // CAN 过滤器标识，需要过滤大于220的ID，保留低4位

#define adc_start(adc_buf)   HAL_ADC_Start_DMA(&hadc1, adc_buf, 1)

#ifdef __cplusplus
}
#endif
