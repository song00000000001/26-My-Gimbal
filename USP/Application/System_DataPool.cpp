/**
 ******************************************************************************
 * @file   System_DataPool.cpp
 * @brief  All used resources are contained in this file.
 ******************************************************************************
 * @note
 *  - User can define datas including variables ,structs ,and arrays in
 *    this file, which are used in deffrient tasks or services.
 **/
#include "internal.h"

/* RTOS Resources ------------------------------------------------------------*/
/* Queues */
QueueHandle_t USART_TxPort; //	串口发送队列
QueueHandle_t USART_RxPort; // 串口接收队列
// StreamBufferHandle_t USART_TxBuffer; // 串口发送缓冲区
QueueHandle_t Param_RxPort;  //调参板接收队列  
QueueHandle_t CAN1_TxPort;  //	can1 发送队列
QueueHandle_t CAN1_RxPort;  //	can1 接收队列
QueueHandle_t CAN2_TxPort;  //	can2 发送队列
QueueHandle_t CAN2_RxPort;  //	can2 接收队列
#if USE_SRML_FS_I6X
QueueHandle_t FS_I6X_QueueHandle; //	FS_I6X（串口） 接收队列
#endif
/* Semaphores */

/* Mutexes */
#if USE_SRML_FS_I6X
SemaphoreHandle_t FS_I6X_mutex; //	FS_I6X互斥量
#endif

//openlog互斥量
SemaphoreHandle_t OpenLog_mutex;

/* Notifications */

/* Other Resources -----------------------------------------------------------*/
#if USE_SRML_VIRTUAL_COM
uint8_t VirtualCom_Rx_Buff[VIRTUALCOM_RX_BUFFER_SIZE];
#endif

#if USE_SRML_MPU6050
__CCM mpu_rec_s mpu_receive; // mpu6050数据
#endif

#if USE_SRML_FS_I6X
__CCM FS_I6X_Classdef FS_I6X; // 遥控器FS_I6X类
#endif

#if USE_SRML_FS_I6X
__CCM FS_I6X_Classdef remote;
#endif // !USE_SRML_FS_I6X

#if USE_SRML_REFEREE
__SRAM referee_Classdef Referee;
#endif
#if USE_SRML_BMI088
/** TODO:  25赛季完全删除，停止对第二代解算库支持*/
// /** BMI088传感器驱动模块 */
// BMI088SensorClassdef BMI088 = BMI088SensorClassdef(&hspi1, BMI088_ACC_NSS_GPIO_Port, BMI088_ACC_NSS_Pin, BMI088_GYRO_NSS_GPIO_Port, BMI088_GYRO_NSS_Pin);
// /** BMI088解算模块 */
// MahonyFilterClassdef BMI088_AHRS;
#endif


/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
