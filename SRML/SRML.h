/**
  ******************************************************************************
  * @file    SRML.h
  * @brief   Header to include the whole library.
  * @version 0.0.1
  ******************************************************************************
  * MOST IMPORTANTLY, this library is not open source for developers from other
  * schools' robot team currently. Plz make sure your code is under your supervision.
  * 
  * Thank for @mannychen @HuanpengLu and other pioneers who have put forward such 
  * architechure, and the endeavors of all developers.
  * 
  * By downloading, copying, installing or using the software you agree to this license.
  * If you do not agree to this license, do not download, install,
  * copy or use the software.
  * 
  *                          License Agreement
  *                For SCUT RobotLab Middleware Layer Library
  * 
  * Copyright (c) 2019 - ~, SCUT RobotLab Development Team, all rights reserved.
  * 
  * This file includes all of the headers of SRML.
  * 
  * Before using this library, plz make sure that u have read the README document
  * carefully,  
  *    @note
  *     - Plz do not modifiy this file(Except for developer).
  *     - Plz remember to update the version number.
  */
#ifndef _SRML_H_
#define _SRML_H_
#pragma once
/** @addtogroup SRML
  * @{
  */

#include "srml_std_lib.h"
#include "srml_config.h"
/* Includes ------------------------------------------------------------------*/
/**
  * @brief Include module's header file
  */

/* Drivers ----------------------------------------------------*/

/* Components header begin */
#if USE_SRML_CAN
  #include "Drivers/Components/drv_can.h"
#endif
#if USE_SRML_FLASH
  #include "Drivers/Components/drv_flash.h"
#endif
#if USE_SRML_I2C
  #include "Drivers/Components/drv_i2c.h"
#endif
#if USE_SRML_SPI
  #include "Drivers/Components/drv_spi.h"
#endif
#if USE_SRML_TIMER
  #include "Drivers/Components/drv_timer.h"
  #include "Drivers/Components/SRML_Timer.h"
#endif
#if USE_SRML_UART
  #include "Drivers/Components/drv_uart.h"
#endif
#if USE_SRML_VIRTUAL_COM
  #include "Drivers/Components/drv_VirtualCom.h"
#endif
/* Components header end */

/* Devices header begin */
#if USE_SRML_MOTOR_DJI
  #include "Drivers/Devices/Motor_Dji/motor_dji.h"
#endif
#if USE_SRML_MF9025_V2
  #include "Drivers/Devices/MF9025_V2/MF9025_V2.h"
#endif
#if USE_SRML_HT04
  #include "Drivers/Devices/HT_04/HT04.h"
#endif
#if USE_SRML_MOTOR_DM
  #include "Drivers/Devices/Motor_DM/motor_dm.h"
#endif
#if USE_SRML_CYBER_GEAR
  #include "Drivers/Devices/Motor_CyberGear/Motor_CyberGear.h"
#endif
#if USE_SRML_LPMS_BE2
  #include "Drivers/Devices/Lpms_Be2/LPMS_BE2.h"
#endif
#if USE_SRML_MPU6050
  #include "Drivers/Devices/MPU6050/mpu6050.h"
#endif
#if USE_SRML_BMI088
  /** TODO: 待BMI088库测试完毕，改为库目录*/
  #if USE_SRML_STATISTICS == 0 
  #error BMI088 needs statictics library to run
  #endif
#endif
#if USE_SRML_DR16
  #include "Drivers/Devices/DR16/dr16.h"
#endif
#if 1
  #include "Drivers/Devices/FS_I6X/FS_I6X.h"
#endif
#if USE_SRML_REFEREE
  #include "Drivers/Devices/referee/referee.h"
#endif
#if USE_SRML_BMX055
  #include "Drivers/Devices/BMX055/BMX055_config.h"
#endif
#if USE_SRML_FATFS
  #include "Drivers/Devices/Flash/FATFS/diskio.h"
#endif
#if USE_SRML_W25Qx
  #include "Drivers/Devices/Flash/W25Qx.h"
#endif

#if USE_SRML_MOTOR_AK80
  #include "Drivers/Devices/motor_AK80/motor_AK80.h"
#endif

#if USE_SRML_VSEC
  #include "Drivers/Devices/VSEC/VSEC.h"
#endif
/* Devices header end */


/* Middlewares -----------------------------------------------*/
/* Algorithms header begin */
#if USE_SRML_ABS_LIB
  #include "Middlewares/Abstract_class_library/abstractIMU.h"
  #include "Middlewares/Abstract_class_library/abstractMotor.h"
#endif
#if USE_SRML_FILTER
  #include "Middlewares/Algorithm/Filters/filters.h"
  #include "Middlewares/Algorithm/Filters/SecondButterworthLPF.h"
#endif
#if USE_SRML_KALMAN
  #include "Middlewares/Algorithm/KalmanFilter/kalman_filter.h"
#endif
#if USE_SRML_PID
  #include "Middlewares/Algorithm/PID/PID.h"
#endif
#if USE_SRML_DIFF_CALCULATER
  #include "Middlewares/Algorithm/DiffCalculator/DiffCalculator.h"
#endif
#if USE_SRML_MATH
  #include "Middlewares/Algorithm/Math/PX4_math.h"
#endif
#if USE_SRML_STATISTICS
  #include "Middlewares/Algorithm/Statistics/statistics.h"
#endif
/* Algorithms header end */

/* Modules header begin */
#if USE_SRML_DIGITAL_POWER
  #include "Middlewares/Module/Digital_Power/digital_Power.h"
  #include "Middlewares/Module/Digital_Power/DP_power_ctrl.h"
#endif
#if USE_SRML_DIGITAL_POWER_V2
  #include "Middlewares/Module/Digital_Power_V2/Digital_Power_V2.h"
  #include "Middlewares/Module/Digital_Power/DP_power_ctrl.h"
#endif
#if USE_SRML_MECENUM_CHASSIS
  #include "Middlewares/Module/Chassis/MecenumChassis.h"
#endif
#if USE_SRML_CHASSIS
  #include "Middlewares/Module/Chassis/chassis.h"
#endif
#if USE_SRML_MOTOR_CTRL
  #include "Middlewares/Module/Motor_Ctrl/motor_ctrl.h"
#endif
#if USE_SRML_POW_CTRL_24
  #include "Middlewares/Module/Power_Ctrl_24/Power_Ctrl_24.h"
#endif
#if USE_SRML_OLD_POW_CTRL
  #include "Middlewares/Module/Old_Power_Ctrl/power_ctrl.h"
#endif
/* Modules header end */

/* Protocols header begin */
#if USE_SRML_SERIAL_LINE_IP
  #include "Middlewares/Protocol/serial_line_ip.h"
#endif
/* Protocols header end */

/* Utilities header begin */
#if USE_SRML_ASUWAVE_MONITOR
  #include "Middlewares/Utility/AsuwaveMonitor/AsuwaveMonitor.hpp"
#endif
#if USE_SRML_VOFA_MONITOR
  #include "Middlewares/Utility/VofaMonitor/VofaMonitor.h"
#endif
#if USE_SRML_UPPER_MONITOR
  #include "Middlewares/Utility/UpperMonitor/UpperMonitor.h"
#endif
#if USE_SRML_LIST
  #include "Middlewares/Utility/linux_list.h"
#endif
#if USE_SRML_MYASSERT
  #include "Middlewares/Utility/my_assert.h"
#endif
#if USE_SRML_SYSANALYSIS
  #include "Middlewares/Utility/sys_analysis.h"
#endif
#if USE_SRML_SYSLOG
  #include "Middlewares/Utility/sys_log.h"
#endif
#if USE_SRML_EASYLOG
  #include "Middlewares/Utility/Easylogger/elog.h"
#endif
/* Utilities header end */
#endif /* _SRML_H_ */
/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
