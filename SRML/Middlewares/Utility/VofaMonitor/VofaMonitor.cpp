/**
  ******************************************************************************
  * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
  * @file    VofaMonitor.cpp
  * @author  ZengXilang chenmoshaoalen@126.com
  * @brief
  * @date    2023/10/05 13:49:01
  * @version 1.0
  * @par Change Log:
  * <table>
  * <tr><th>Date <th>Version <th>Author <th>Description
  * <tr><td>2023-10-05 <td> 1.0.0 <td>zxl <td>Creator
  * <tr><td>2023-10-10 <td> 1.1.0 <td>yjh <td>Updater
  * </table>2023-10-17 <td> 1.1.1 <td>lrc <td>Modify the instructions for use
  *
  ==============================================================================
                               How to use this Lib
  ==============================================================================
    @note
      -# 配置好串口与相应DMA
      -# 调用VofaMonitor::setDatas(uint8_t firstindex, T... dataArg),传入起始通道编号(0~9)和需要传输的数据
      -# 调用VofaMonitor::send(uint8_t uart_id)发送数据,传参为串口id号
      -# 推荐调用周期5ms、串口波特率460800（可以同时发送更多数据）
      -# 更多信息请跳转https://scutrobotlab.feishu.cn/wiki/NGq0wdbrTiIYiOkAihncmiGen6q?from=from_copylink
    @warning
      -# 默认最大曲线数目设置为10条，可以直接修改VOFAMONITOR_MAX_DATA_NUM，注意串口波特率与发送
         周期即可（过低的发送频率和发送周期会导致丢包）。
      -#
  ******************************************************************************
  * @attention
  *
  * if you had modified this file, please make sure your code does not have many
  * bugs, update the version Number, write dowm your name and the date, the most
  * important is make sure the users will have clear and definite understanding
  * through your new brief.
  *
  * <h2><center>&copy; Copyright (c) 2019 - ~, SCUT-RobotLab Development Team.
  * All rights reserved.</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "VofaMonitor.h"
#if USE_SRML_VOFA_MONITOR

#include "Drivers/Components/drv_uart.h"
/* Configurations ------------------------------------------------------------*/
using namespace VofaMonitor;
/* Messages ------------------------------------------------------------------*/
#if VOFAMONITOR_MAX_DATA_NUM < 0
#error "The maximum amount of "Max_Sent_Data_Num" must be greater than 0."
#else
//#warning "Please pay attention to the baud rate of the serial port (UART/USART) to avoid data loss. "
#endif
/* Private variables ---------------------------------------------------------*/
namespace VofaMonitor
{
  __CCM uint8_t DataNum = 0;                                     // 记录当前需要传输的数据量
  __SRAM Type_change_t SendDataPool[VOFAMONITOR_MAX_DATA_NUM + 1]; // 传输数据池
}
/* Private function declarations ---------------------------------------------*/
/* function prototypes -------------------------------------------------------*/
/**
 * @brief 向Vofa上位机发送数据
 *
 * @param uart_id 发送的串口ID号
 */
void VofaMonitor::send(uint8_t uart_id)
{
  static USART_COB TxPack = {1, 0, (uint8_t *)SendDataPool};

  SendDataPool[DataNum + 1].change_u8[0] = 0x00;
  SendDataPool[DataNum + 1].change_u8[1] = 0x00;
  SendDataPool[DataNum + 1].change_u8[2] = 0x80;
  SendDataPool[DataNum + 1].change_u8[3] = 0x7f;

  TxPack.len = sizeof(Type_change_t) * (DataNum + 2);
  TxPack.port_num = uart_id;
  SRML_UART_Transmit_DMA(&TxPack);
  DataNum = 0;
}

#endif /* USE_SRML_VOFA_MONITOR */
/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
