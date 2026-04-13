#pragma once
#include "SRML.h"
#include "tim.h"    
#include "adc.h"

#ifdef __cplusplus
extern "C" {
#endif

#define upper_uart_id 4 // 上位机通信串口
#define motor_uart_id 1 // 电机通信串口 
#define motor_comm_delay_ms 7 // 电机通信周期，38400波特率下理论上4ms，但是发送可以保证4ms，如果要等电机回复需要6~7ms不等。
//7ms下测试丢包率1%~2%
//8ms下测试丢包率0.5%~1%
//认为7ms是一个比较合理的选择，既能保证通信效率又能降低丢包率。后续可以考虑增加重试机制来进一步降低丢包率。

#define STACK_REMAIN_MONITER_ENABLE 0 // 是否启用任务栈剩余监测功能

// 帧头定义
#define FAN_PACKET_HEADER 0xA5
#define CAN_SEND_ID_BASE 0x210  // CAN 发送包头标识
#define CAN_RECEIVE_ID_BASE 0x220 // CAN 接收包头标识
#define CAN_FILTER_ID_MASK 0x7F0 // CAN 过滤器标识，需要过滤大于220的ID，保留低4位

#define adc_start(adc_buf)   HAL_ADC_Start_DMA(&hadc1, adc_buf, 1)

#define motor_delay() vTaskDelay(pdMS_TO_TICKS(motor_comm_delay_ms)) // 电机通信延时，确保电机通信正常

#ifdef __cplusplus
}
#endif
