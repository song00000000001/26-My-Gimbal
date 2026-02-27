/**
  ******************************************************************************
  * @file   APP.cpp
  * @brief  Devices Application running file.
  ******************************************************************************
  * @note
  *  - Before running your devices, just do what you want ~ !
  *  - More devices or using other classification is decided by yourself ~ !
  ===============================================================================
                                    Task List
  ===============================================================================
  * <table>
  * <tr><th>Task Name     <th>Priority          <th>Frequency/Hz    <th>Stack/Byte
  * <tr><td>              <td>                  <td>                <td>
  * </table>
  *
 */

/* Includes ------------------------------------------------------------------*/
#include "config.h"
#include "internal.h"
/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/*Private Function declarations -------------------------------------------------------*/
#if USE_SRML_MPU6050
static void System_Mpu_Init();
#endif
/* Function prototypes -------------------------------------------------------*/
/**
 * @brief  application device initialization task
 */
void System_Device_Init(void)
{
  /* Drivers Init */
  // timer init
#if USE_SRML_TIMER
  Timer_Init(&htim5, USE_MODULE_DELAY);//请确保使用该模式时的定时器中断不归RTOS管辖
  //Timer_Init(&htim5, USE_HAL_DELAY);//此设置语句使得delay_ms_nos()函数调用的是HAL_Delay(),容易造成FreeRTOS进程紊乱
#endif

#if USE_SRML_CAN
  // can init
  CAN_Init(&hcan1, User_CAN1_RxCpltCallback);
  CAN_Init(&hcan2, User_CAN2_RxCpltCallback);

  // CAN1: 达妙电机反馈 0x302
  CAN_Filter_Mask_Config(1, CanFilter_0 | CanFifo_0 | Can_STDID, 0x302, 0x7FF);
  // CAN2: 分控反馈 0x221~0x225 (掩码匹配 0x220 范围)
  CAN_Filter_Mask_Config(2, CanFilter_14 | CanFifo_0 | Can_STDID, 0x220, 0x7F8);
  
#endif
  // uart init
#if USE_SRML_UART
#if USE_SRML_DR16
  Uart_Init(&huart2, NULL, UART2_RX_BUFFER_SIZE, DR16_RxCpltCallback);//遥控DBUS口
#elif USE_SRML_FS_I6X
	Uart_Init(&huart2, NULL, UART2_RX_BUFFER_SIZE, FS_I6X_RxCpltCallback);
#else
  Uart_Init(&huart2, NULL, UART2_RX_BUFFER_SIZE, UART2_RxCpltCallback);
#endif

#if USE_SRML_REFEREE

#else

#endif

  Uart_Init(&huart1, NULL, UART1_RX_BUFFER_SIZE, UART1_RxCpltCallback);//上位机串口
  Uart_Init(&huart6, NULL, UART6_RX_BUFFER_SIZE, UART6_RxCpltCallback);//vofa调试串口
#endif  //#if USE_SRML_UART

#if USE_SRML_VIRTUAL_COM
	VirtualComUserInit(VirtualCom_Rx_Buff, User_VirtualComRecCpltCallback, NULL);
#endif

#if USE_SRML_REFEREE
  Referee.Init(&huart6, Get_SystemTimer);
#endif

#if USE_SRML_MPU6050
  System_Mpu_Init();
#endif

#if USE_SRML_BMI088
  /** TODO:  25赛季完全删除注释部分，停止对老版BMI解算的支持*/
  //BMI088.SetupE2PROM(GPIOB,GPIO_PIN_6,GPIO_PIN_7);
  //BMI088.Init();
  //assert_param(BMI088.SensorErrorCode != IMU_CommunicationError); 
  BMI088_Init();
#endif

#if USE_SRML_EASYLOG
  Easylog_init(4, EASY_UART);
#endif
}

/**
 * @brief application freertos init function.
 */
void System_Task_Init(void)
{
  /* Queue Init */
    CAN1_TxPort = xQueueCreate(6, sizeof(CAN_COB));
    CAN1_RxPort = xQueueCreate(6, sizeof(CAN_COB));
    CAN2_TxPort = xQueueCreate(6, sizeof(CAN_COB));
    CAN2_RxPort = xQueueCreate(6, sizeof(CAN_COB));
    USART_TxPort = xQueueCreate(6, sizeof(USART_COB));
    USART_RxPort = xQueueCreate(6, sizeof(USART_COB));
#if USE_SRML_DR16
    DR16_QueueHandle = xQueueCreate(3, sizeof(USART_COB));
#endif
#if USE_SRML_FS_I6X
    FS_I6X_QueueHandle = xQueueCreate(3, sizeof(USART_COB));
#endif
    /* Semaphore Init */
    /* Mutex Init */
#if USE_SRML_DR16
    DR16_mutex = xSemaphoreCreateMutex();
#endif
    OpenLog_mutex = xSemaphoreCreateMutex();
    /* Task Init */
    Service_Communication_Init();
    Service_Devices_Init();
    Service_Debug_Init();
}

#if USE_SRML_MPU6050
/**
 * @brief 		mpu6050初始化函数
 * @function	确定mpu6050 i2c 通信引脚；
 * @function	mpu6050按给定参数初始化
 */
static void System_Mpu_Init()
{
  /*	配置MPU6050 I2C 引脚	*/
  MPU6050_Config_Pin(GPIOB, GPIO_PIN_6, GPIO_PIN_7);
  /*	配置MPU6050 参数  完成初始化	*/
  MPU6050_Init(&mpu_config, &dmp_config);
  /*	运行陀螺仪自检	*/
  MPU6050_run_self_test(0, 10);
}
#endif
