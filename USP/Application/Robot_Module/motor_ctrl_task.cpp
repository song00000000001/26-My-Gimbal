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
    static bool system_enable_last = true;
    MotorSpeedCtrlOutput speed_ctrl_out = {0};
    
    TickType_t xLastWakeTime_t;
    xLastWakeTime_t = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(motor_comm_delay_ms); // 7ms周期，确保电机通信正常
    vTaskDelay(pdMS_TO_TICKS(1200)); // 等待电机上电并且发完上电指令。
    motor_init(motor_uart_id); // 初始化电机
    gimbal_pid_init(); // 初始化PID控制器

    Debugger.system_enable = true;// 系统使能
    Debugger.spd_feedback_source = false;               // 默认使用电机速度反馈
    Debugger.enable_debug_mode = debug_mtvofa_monitor;  // 默认开启mtvofa监控模式
    Debugger.motor_index = YAW;                     // 默认调试YAW电机

    g_pid_debug[YAW].cascade_enable = MY_PID_SPEED_LOOP_ENABLE; // 默认速度环串电流环
    /**
     * @brief PID参数初始化
     */
    g_pid_debug[YAW].pos={
        .ref=0.0f,
        .out_range=170.0f,
        .integ_range=0.0f,
        .param={
                .kp = 1.0f,
                .ki = 0.0f,
                .kd = 0.0f,
                .kff= 0.0f
        },
        .feature={
            .integ_enable=false,
            .d_split_enable=true
        }
        
    };
    g_pid_debug[YAW].spd={
        .ref=0.0f,
        .out_range=0.6f,
        .integ_range=0.1f,
        .param={
                .kp = 2.0f,
                .ki = 1.0f,
                .kd = 0.2f,
                .kff= 0.2f
        },
        .feature={
            .integ_enable=true,
            .d_split_enable=true,
            .integ_split_enable=true
        }
    };
    MyPid_SetIntegSplitThreshold(g_pid_debug[YAW].spd, 10.0f);
    #if USE_MCU_CURRENT_LOOP
    //电机开环，跑自己的电流环
    Debugger.motor_mode = MotorMode::OPEN_LOOP;
    g_pid_debug[YAW].cur={
        .ref=0.0f,
        .out_range=4.0f,
        .integ_range=4.0f,
        .param={
                .kp = 0.0f,
                .ki = 300.0f,
                .kd = 0.0f,
                .kff= 0.0f
        },
        .feature={
            .integ_enable=true,
            .d_split_enable=true
        }
    };
    #else
    //电机电流环，自己的电流环直接设置kff=1即可。
    Debugger.motor_mode = MotorMode::CURRENT_LOOP;     // 默认电流环模式
    g_pid_debug[YAW].cur={
        .ref=0.0f,
        .out_range=0.6f,
        .integ_range=0.0f,
        .param={
                .kp = 0.0f,
                .ki = 0.0f,
                .kd = 0.0f,
                .kff= 1.0f
        },
    };
    #endif

    for (int i = 0; i < MOTOR_COUNT; ++i) {
        motor_speed_ctrl_reset(&g_speed_ctrl[i]);
    }

    for (;;)
    {
        /**
         * @brief 计算控制输出
         */
        const float yaw_speed_feedback = Debugger.spd_feedback_source ? g_imu_gyro_dps[YAW] : g_motors[YAW].getSpeed();
        const float yaw_current_feedback = g_motors[YAW].getCurrent();
        const float speed_ref = g_pid_debug[YAW].spd.ref;

        // 速度环原始输出：只做速度PI，不混入补偿
        MyPid_SetDebugParam(&g_pid[YAW].spd, &g_pid_debug[YAW].spd);
        const float iq_ref_base = MyPid_Calc(&g_pid[YAW].spd, speed_ref, yaw_speed_feedback);

        // 起转状态机 + 起转补偿 + 运行摩擦前馈
        motor_speed_ctrl_update(&g_speed_ctrl[YAW],
                                &g_speed_ctrl_param,
                                speed_ref,
                                yaw_speed_feedback,
                                iq_ref_base,
                                motor_comm_delay_ms,
                                &speed_ctrl_out);

        if (speed_ctrl_out.startup_timeout) {
            MyPid_Reset(&g_pid[YAW].spd);
            #if USE_MCU_CURRENT_LOOP
            MyPid_Reset(&g_pid[YAW].cur);
            #endif
        }

        float drive_cmd = 0.0f;
        #if USE_MCU_CURRENT_LOOP
        MyPid_SetDebugParam(&g_pid[YAW].cur, &g_pid_debug[YAW].cur);
        drive_cmd = MyPid_Calc(&g_pid[YAW].cur, speed_ctrl_out.iq_cmd, yaw_current_feedback);
        #else
        drive_cmd = speed_ctrl_out.iq_cmd;
        #endif
       
        /**
         * @brief 发送控制指令
         */
        if(!Debugger.system_enable){
            vTaskDelayUntil(&xLastWakeTime_t, xFrequency);
            motor_disable();
        }
        else{
            // vTaskDelayUntil(&xLastWakeTime_t, xFrequency);
            // g_motors[PITCH].sendCurrentCtrl(gimbal_pid_spd[PITCH].data.out/10.0f);
            vTaskDelayUntil(&xLastWakeTime_t, xFrequency);
            g_motors[YAW].sendCurrentCtrl(drive_cmd);
        }
        
        /**
         * @brief 系统使能状态切换检测
         */
        if(Debugger.system_enable && !system_enable_last){
            //系统从失能变为使能，重新初始化电机
            motor_init(motor_uart_id);
        }
        system_enable_last = Debugger.system_enable;
        
        if(Debugger.motor_index == YAW){
            motor_observer = &g_motors[YAW];
            pid_observer = &g_pid[YAW];
            pid_debug_observer = &g_pid_debug[YAW];
        }
        else{
            motor_observer = &g_motors[PITCH];
            pid_observer = &g_pid[PITCH];
            pid_debug_observer = &g_pid_debug[PITCH];
        }

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
        
        g_motors[i].setPortNum(port_id);// 设置串口ID
        motor_delay();
        g_motors[i].sendEnableCmd(true); // 使能
        motor_delay();
        g_motors[i].sendModeCmd(Debugger.motor_mode);
        motor_delay();
    }
}

void Mypid_init_m0603a_pos(MyPid *pid)
{
    MyPid_Init(pid, MY_PID_MODE_POSITION,1.0f,motor_comm_delay_ms/1000.0f);
    // MyPid_SetLimit(pid,
    //                 0.0f, 0.0f,        // out 限幅
    //                 0.0f, 0.0f,        // 积分限幅
    //                 0.0f, 0.0f         // 增量输出限幅
    //                 );            
    // MyPid_SetIntegSplitThreshold(pid, 0.0f);
}

void Mypid_init_m0603a_spd(MyPid *pid)
{
    MyPid_Init(pid, MY_PID_MODE_POSITION,0.001f,motor_comm_delay_ms/1000.0f);
}

void Mypid_init_m0603a_cur(MyPid *pid)
{
    MyPid_Init(pid, MY_PID_MODE_POSITION,1.0f,motor_comm_delay_ms/1000.0f);
}

void gimbal_pid_init(void)
{
    Mypid_init_m0603a_pos(&g_pid[YAW].pos);
    Mypid_init_m0603a_spd(&g_pid[YAW].spd);
    Mypid_init_m0603a_cur(&g_pid[YAW].cur);
}

/**
 * @brief 电机失能
 * @details 该函数负责电机的失能配置，通常在系统关闭或者需要紧急停止电机时调用。需要确保在调用该函数后，电机不会再响应任何控制指令。
 */
static void motor_disable()
{
    for (int i = 0; i < MOTOR_COUNT; i++) {
        motor_delay();
        g_motors[i].sendEnableCmd(false); // 失能
    }
}


/** 
  * @brief 电机测试总结
  * @details
电机用手使劲转，产生比较大的反馈电流（大于0.6a）后,自己会退化成开环。而且反馈的模式不会更新。重新失能使能发一次模式就好了。
开环下需要更大的输出才能起转，且起转后速度和电流的关系不太线性。电流环模式下，起转电流比较小，且速度和电流的关系比较线性。后续可以识别下开环特征自动重新使能。

# 开环模式
输出0.90基本能克服起转静摩擦。
输出0.8，固定时反馈是80ma=0.08a左右，相差10倍。
转起来后就在50~60ma左右。手动正反转能跑到800ma/-337ma。
25v ，给4，反馈最大1260，消耗0.9a。静态0.062a。最快410rpm。
12v，给4，最大590，消耗0.47a。静态0.095a。最快180rpm。

| 输出/反馈电流平均，转速平均 | ma  | rpm     |
| -------------- | --- | ------- |
| 0.9            | 50  | 15.19   |
| 1              | 55  | 20.694  |
| 2              | 67  | 74.02   |
| 3              | 71  | 127.762 |
| 4              | 80  | 182.11  |

- 测试输出固定0.9，电机固定（堵转），电压12v，电流反馈平均值0.09，电压逐渐上升到24v，反馈一路上升到0.21。
- 感觉在12v下，可以近似认为输出0.9就是静态给0.09ma的输出。也可能是电压或者力矩。开环大概可以这么计算吧。
# 电流环模式
- 电机固定（堵转）。（0.1~0.5输出下，电压从10v~25v下响应基本一致）
	- 输出一个阶跃信号，从0到0.1，反馈电流均值0.1a，响应91ms，无超调，整体平稳，抖动幅度在0.0048a以内。
	- 从0.1加到0.2，再加到0.3,再加到0.4测试响应和误差基本一致。电流环工作良好。
	- 0.5时，响应一致，但是出现14ms超调，幅度0.0307a。稳态误差0.0044a。
	- 0.6时，输出饱和，反馈均值在0.576a。（12v下饱和）
- 测试最大起转输出。
	- 一般从0给0.09的阶跃能直接转起来。
	- 但是从0给0.07的阶跃，很久也不会转。然后0.07每次加0.1，最大在0.11才开始转。
- 测试维持最小转速输出
	- 给0.052，稳态误差0.0052a，用手辅助起转后（大于30rpm保持几百毫秒左右），能一直转，转速区间21~44rpm。用手轻轻辅助起转（起转速度大于22rpm)，转了2圈会停。最大转速也有44rpm。用手轻碰（起转速度小于20rpm），一般转不到1/2圈就停了。
- 给0.06，并且轻轻触碰后，就能从0加速到172rpm左右。耗时7770ms。加速过程速度一直有反复震荡，震荡幅度基本都在9rpm左右，震荡周期基本在150ms左右。整体波形像是对数上升曲线叠加了正弦。
- 给0.068，并且轻轻触碰后，就能从0加速到180rpm左右。
- 根据以上测试结果，帮我设计起转死区代码思路和前馈实现思路以及前馈参数大致范围。
*/