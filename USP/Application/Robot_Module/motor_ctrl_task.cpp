#include "internal.h"
#include "global_data.h"
#include "robot_config.h"
#include "comm_protocal.h"

static void motor_init(uint8_t port_id);
static void motor_disable();
static void gimbal_pid_init(void);

/**
 * @brief 电机控制任务
 */
void task_motor_ctrl(void *arg)
{
    TickType_t xLastWakeTime_t;
    xLastWakeTime_t = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(motor_comm_delay_ms); // 7ms周期，确保电机通信正常
    vTaskDelay(pdMS_TO_TICKS(1200)); // 等待电机上电并且发完上电指令。
    motor_init(motor_uart_id); // 初始化电机
    gimbal_pid_init(); // 初始化PID控制器
    Debugger.angle_loop_enable = true; // 默认开启角度环串速度环的调试模式
    Debugger.spd_feedback_source = false; // 默认使用电机速度反馈
    Debugger.enable_debug_mode = debug_mtvofa_monitor; // 默认开启mtvofa监控模式
    Debugger.spd_target_rpm = 0.0f; // 速度环单独调试时的目标速度，单位为RPM
    Debugger.system_enable = true; // 系统使能
    for (;;)
    {
        /**
         * @brief 先请求反馈
         * 暂时注释掉，目前不需要获取电机的里程和位置等额外反馈数据，后续如果需要再打开。
         */
        // vTaskDelayUntil(&xLastWakeTime_t, xFrequency);
        // gimbal_motors[YAW].sendQueryExtraCmd(); // 请求额外反馈，获取里程和精确位置等信息
        
        /**
         * @brief 计算控制输出
         */
        for(int i=0;i<MOTOR_COUNT;i++){
            MyPid_Calc(&gimbal_pid_pos[i],hold_angle_deg[i],imu_angle_deg[i]);
            /**
             * @brief 速度环的计算
             * 由于云台是电机直驱，并没有经过减速，所以速度反馈数据既可以从电机取，也可以从IMU的角速度数据取。
             * 电机的速度数据会有一点毛刺和噪声，但总体还行；IMU的角速度数据相对更平滑，但有延迟。
             */
            float gimbal_speed_feedback = Debugger.spd_feedback_source ? imu_gyro_dps[i] : gimbal_motors[i].getSpeed() ;
            if(Debugger.angle_loop_enable){
                MyPid_Calc(&gimbal_pid_spd[i],gimbal_pid_pos[i].data.out,gimbal_speed_feedback);
            }
            else{
                MyPid_Calc(&gimbal_pid_spd[i],Debugger.spd_target_rpm,gimbal_speed_feedback);
            }
        }
       
        /**
         * @brief 发送控制指令
         */
        
        if(!Debugger.system_enable){
            vTaskDelayUntil(&xLastWakeTime_t, xFrequency);
            motor_disable();
        }
        else{
            vTaskDelayUntil(&xLastWakeTime_t, xFrequency);
            gimbal_motors[PITCH].sendCurrentCtrl(gimbal_pid_spd[PITCH].data.out); 
            vTaskDelayUntil(&xLastWakeTime_t, xFrequency);
            gimbal_motors[YAW].sendCurrentCtrl(gimbal_pid_spd[YAW].data.out);
        }
        
        /**
         * @brief 系统使能状态切换检测
         */
        static bool system_enable_last = true;
        if(Debugger.system_enable && !system_enable_last){
            //系统从失能变为使能，重新初始化电机
            motor_init(motor_uart_id);
        }
        system_enable_last = Debugger.system_enable;
        
        #if STACK_REMAIN_MONITER_ENABLE
        StackWaterMark_Get(motor_ctrl);
        #endif
    }
}

/**
 * @brief 电机初始化
 * @param port_id 串口ID，例如 1 表示 USART1
 * @details 该函数负责电机的初始化配置，包括注册发送函数和设置串口ID。需要在使用电机之前调用一次，确保驱动内部能够正确发送指令包
 * @todo
 * 这里的4ms延时是因为电机最快通信周期为4ms。
 * 第一次发送指令后需要等4ms才能发送下一条指令，否则可能会被电机拒绝或者丢包。
 * 考虑在communication层按照4ms的周期从队列里取出指令包发送给电机。
 * 但是手册只写了单个电机的通信周期，两个电机同时占用总线通信时是否需要更长的周期还不确定，后续需要测试。
 */
static void motor_init(uint8_t port_id)
{
    // 电机初始化逻辑
    // 1. 注册发送函数指针，供驱动内部调用发送指令包
    BenMoMotor::registerSendFunction(send_motor_packet);
    // 2. 设置发送使用的串口ID
    for (int i = 0; i < MOTOR_COUNT; i++) {
        
        gimbal_motors[i].setPortNum(port_id);// 设置串口ID
        motor_delay();
        gimbal_motors[i].sendEnableCmd(true); // 使能
        motor_delay();
        gimbal_motors[i].sendModeCmd(MotorMode::OPEN_LOOP); // 切换到电流环模式
        motor_delay();
    }
}

void gimbal_pid_init(void)
{
    vTaskDelay(1000);//等imu输出
    MyPid_Init(&gimbal_pid_pos[YAW], MY_PID_MODE_POSITION, 1.2f,100.0f, 0.0f,motor_comm_delay_ms/1000.0f);//dt根据任务周期设置，这里是7ms
    // 输出为电机转速，电机空载转速400rpm，额定100rpm，极值参考驱动函数。
    MyPid_SetLimit(&gimbal_pid_pos[YAW],
                    -170.0f,   170.0f,      // target_accum / out 限幅
                    -10.0f, 10.0f,       // 积分限幅
                    0,  0);              //特殊模式下的增量限幅，这里无意义
    MyPid_SetIntegSplitThreshold(&gimbal_pid_pos[YAW], 10.0f); // 误差超过5度时暂停并清空积分，避免大误差引起的积分风暴。
    gimbal_pid_pos[YAW].integ_enable = ture;

    MyPid_Init(&gimbal_pid_spd[YAW], MY_PID_MODE_POSITION, 0.06f, 0.01f, 0.0f, motor_comm_delay_ms/1000.0f);//dt根据任务周期设置，这里是7ms
    MyPid_SetLimit(&gimbal_pid_spd[YAW],
                    -4.0f,   4.0f,      // target_accum / out 限幅
                    -1.0f, 1.0f,       // 积分限幅
                    0,  0);              //特殊模式下的增量限幅，这里无意义
    MyPid_SetIntegSplitThreshold(&gimbal_pid_spd[YAW], 30.0f); 
    
    MyPid_Init(&gimbal_pid_pos[PITCH], MY_PID_MODE_POSITION, 1.0f,0.0f, 0.0f,motor_comm_delay_ms/1000.0f);//dt根据任务周期设置，这里是7ms
    // 输出为电机转速，电机空载转速400rpm，额定100rpm，极值参考驱动函数。
    MyPid_SetLimit(&gimbal_pid_pos[PITCH],
                    -170.0f,   170.0f,      // target_accum / out 限幅
                    0.0f, 0.0f,       // 积分限幅
                    0,  0);              //特殊模式下的增量限幅，这里无意义
    MyPid_SetIntegSplitThreshold(&gimbal_pid_pos[PITCH], 5.0f); // 误差超过5度时暂停并清空积分，避免大误差引起的积分风暴。
    gimbal_pid_pos[PITCH].integ_enable = false; // 角度环暂时不启用积分

    MyPid_Init(&gimbal_pid_spd[PITCH], MY_PID_MODE_POSITION, 0.003f, 0.01f, 0.0f, motor_comm_delay_ms/1000.0f);//dt根据任务周期设置，这里是7ms
    MyPid_SetLimit(&gimbal_pid_spd[PITCH],
                    -4.0f,   4.0f,      // target_accum / out 限幅
                    -1.0f, 1.0f,       // 积分限幅
                    0,  0);              //特殊模式下的增量限幅，这里无意义
    MyPid_SetIntegSplitThreshold(&gimbal_pid_spd[PITCH], 30.0f); 
}

/**
 * @brief 电机失能
 * @details 该函数负责电机的失能配置，通常在系统关闭或者需要紧急停止电机时调用。需要确保在调用该函数后，电机不会再响应任何控制指令。
 */
static void motor_disable()
{
    for (int i = 0; i < MOTOR_COUNT; i++) {
        motor_delay();
        gimbal_motors[i].sendEnableCmd(false); // 失能
    }
}

