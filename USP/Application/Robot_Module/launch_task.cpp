#include "internal.h"
#include "global_data.h"
#include "launcher_driver.h"
#include "robot_config.h"

/* 自检掩码定义 */
#define MASK_DELIVER_L  (1 << 0)
#define MASK_DELIVER_R  (1 << 1)
#define MASK_IGNITER    (1 << 2)
// 如果有Yaw或其他，继续往后加
// #define MASK_YAW_L   (1 << 3) 

// 定义全部通过的目标值 (0b111 = 0x07)
#define MASK_ALL_PASSED (MASK_DELIVER_L | MASK_DELIVER_R | MASK_IGNITER)

/* --- 1. 全局对象实例化 --- */
// 定义全局机器人状态 (Cmd, Flag, Status)
Robot_Ctrl_t Robot; 

// 实例化驱动 (假设 ID 为 CAN1_1, CAN1_2, CAN2_1，请修改为你实际的 ID)
Launcher_Driver Launcher(1, 2, 1); 


/* --- 2. 辅助变量 --- */
static uint32_t stick_hold_timer = 0; // 用于长按判断
static bool is_combo_active = false;  // 组合键激活标志

/* --- 3. 输入解析函数 --- */
static void Update_Input()
{
    // 本地数据快照,解决数据一致性和锁性能问题
    DR16_Snapshot_t rc_data;
    // 假设 GetStatus 内部不加锁，或者如果需要锁，在这里加一次即可
    // xSemaphoreTake(DR16_mutex, portMAX_DELAY); 
    rc_data.Status = DR16.GetStatus();
    rc_data.S1 = DR16.GetS1();
    rc_data.S2 = DR16.GetS2();
    rc_data.RX_Norm = DR16.Get_RX_Norm();
    rc_data.RY_Norm = DR16.Get_RY_Norm();
    rc_data.LX_Norm = DR16.Get_LX_Norm();
    rc_data.LY_Norm = DR16.Get_LY_Norm();
    // xSemaphoreGive(DR16_mutex);

    // [1] 系统基础指令
    Robot.Cmd.sys_enable = (rc_data.S1 != SW_UP); // 左拨杆非上 -> 使能
    Robot.Cmd.auto_mode  = (rc_data.S1 == SW_DOWN); // 左拨杆下 -> 自动
    
    // [2] 手动/自检指令
    Robot.Cmd.manual_override = (rc_data.S2 == SW_DOWN); // 右拨杆下 -> 手动接管
    Robot.Cmd.skip_check      = (rc_data.S2 == SW_MID);  // 右拨杆中 -> 跳过自检

    // [3] 摇杆组合键逻辑 (长按 1s 切换模式)
    // 定义阈值
    const float THRESHOLD = 0.8f;
    float ly = rc_data.LY_Norm;
    float lx = rc_data.LX_Norm;
    
    // 判断是否处于特定区域
    bool region_2_burst =    (ly > THRESHOLD && lx < -THRESHOLD);  // 上+左
    bool region_4_burst =    (ly > THRESHOLD && lx > THRESHOLD);   // 上+右
    bool region_outpost =    (ly < -THRESHOLD && lx < -THRESHOLD); // 下+左
    bool region_base =       (ly < -THRESHOLD && lx > THRESHOLD);  // 下+右

    if (region_2_burst || region_4_burst || region_outpost || region_base) {
        if (!is_combo_active) {
            stick_hold_timer = xTaskGetTickCount();
            is_combo_active = true;
        } 
        else {
            // 保持了 1000ms
            if ((xTaskGetTickCount() - stick_hold_timer) > 1000) {
                if (region_2_burst) Robot.Cmd.burst_num = BURST_2;
                if (region_4_burst) Robot.Cmd.burst_num = BURST_4;
                if (region_outpost) Robot.Cmd.target = TARGET_OUTPOST;
                if (region_base)    Robot.Cmd.target = TARGET_BASE;
                
                // 重置以免重复触发，或者保持直到摇杆归位,奇怪的逻辑,好像是6s内有效
                stick_hold_timer = xTaskGetTickCount() + 5000;
            }
        }
    } 
    else {
        is_combo_active = false;
    }

    // [4] 手动控制量 (仅在手动模式下有效)
    if (Robot.Cmd.manual_override) {
        // 映射右摇杆到 Pitch (丝杆) 和 Yaw
        Robot.Cmd.manual_pitch_inc = DR16.Get_RY_Norm() * 0.5f; // 调整系数
        Robot.Cmd.manual_yaw_inc   = DR16.Get_RX_Norm() * 0.1f;
        
        // 发射指令 (手动模式下 S1 下拨也算，或者定义别的键)
        Robot.Cmd.fire_command = (DR16.GetS1() == SW_DOWN); 
    } else {
        Robot.Cmd.manual_pitch_inc = 0;
        Robot.Cmd.manual_yaw_inc = 0;
        Robot.Cmd.fire_command = false; // 自动模式下的发射由后面逻辑决定
    }
}

/* --- 4. 发射流程子状态机 --- */
// 定义子状态
typedef enum {
    FIRE_IDLE,
    FIRE_PULLING,    // 下拉
    FIRE_LATCHING,   // 挂机 (等待挂稳)
    FIRE_RETURNING,  // 回升
    FIRE_SHOOTING,   // 开火 (舵机)
    FIRE_COOLDOWN    // 冷却
} Fire_State_e;

static void Run_Firing_Sequence()
{
    static Fire_State_e fire_state = FIRE_IDLE;
    static uint32_t state_timer = 0;
    
    // 定义位置常量 (请根据实际调试修改)
    const float POS_BUFFER = -200.0f; 
    const float POS_BOTTOM = -640.0f;

    switch (fire_state)
    {
    case FIRE_IDLE:
        // 确保滑块在缓冲区
        Launcher.set_deliver_target(POS_BUFFER);
        Launcher.fire_reset(); // 关舵机
        
        // 触发条件：收到指令 且 视觉瞄准到位(可选)
        // 这里假设 Cmd.fire_command 在自动模式下由视觉改写，或者简单的连发逻辑
        if (Robot.Cmd.fire_command) {
            fire_state = FIRE_PULLING;
        }
        break;

    case FIRE_PULLING:
        // 1. 锁住丝杆 (防止堵转) -> 对应 Driver 里的 MODE_LOCKED (输出0电流)
        //    我们需要在 Launcher 类加一个接口，或者直接在这里利用状态位
        //    为了简单，我们假设 Launcher 内部逻辑：
        //    如果我们要下拉，我们不改变丝杆目标，但因为结构问题，建议丝杆卸力
        //    【重要】这里调用我们在 Driver 里新加的特殊模式接口(如果有)
        //    或者简单地：不更新丝杆目标，让它保持原位(有电流)或者设为0电流
        
        // 2. 滑块全速下拉
        Launcher.fire_reset();//锁止扳机,准备扣上滑台
        Launcher.set_deliver_target(POS_BOTTOM);
        
        // 3. 判断到位
        if (Launcher.is_deliver_at_target()) {
            state_timer = xTaskGetTickCount();
            fire_state = FIRE_LATCHING;
        }
        break;

    case FIRE_LATCHING:
        //300ms是给扳机舵机动作时间,实际上在FIRE_PULLING的过程中有足够时间动作,
        //但是考虑到机械惯性和安全，保留等待
        if ((xTaskGetTickCount() - state_timer) > 300) {
            fire_state = FIRE_RETURNING;
        }
        break;

    case FIRE_RETURNING:
        // 滑块回缓冲区
        Launcher.set_deliver_target(POS_BUFFER);
        
        // 判断到位
        if (Launcher.is_deliver_at_target()) {
            // 此时滑块已避让，可以开火
            fire_state = FIRE_SHOOTING;
            state_timer = xTaskGetTickCount();
        }
        break;

    case FIRE_SHOOTING:
        // 舵机动作
        Launcher.fire_trigger();
        
        // 等待发射完成 (例如 500ms)
        if ((xTaskGetTickCount() - state_timer) > 500) {
            Robot.Status.dart_count++; // 计数+1
            fire_state = FIRE_COOLDOWN;
            state_timer = xTaskGetTickCount();
        }
        break;

    case FIRE_COOLDOWN:
        Launcher.fire_reset(); // 舵机复位
        
        // 冷却时间 (例如 1000ms)
        if ((xTaskGetTickCount() - state_timer) > 1000) {
            // 判断是否继续连发
            // if (Robot.Status.dart_count < Robot.Cmd.burst_num) ...
            
            fire_state = FIRE_IDLE;
        }
        break;
    }
    
    // 异常打断：如果突然切出手制动，重置状态机
    if (!Robot.Cmd.auto_mode) {
        fire_state = FIRE_IDLE;
    }
}


/**
 * @brief 发射主控任务 (大脑)
 */
void LaunchCtrl(void *arg)
{
    //can发送的包
    Motor_CAN_COB Tx_Buff;
    // ================== 1. 初始化阶段 ==================
    
    // 初始化驱动
    Launcher.init();
    
    // 绑定限位开关读取函数 (请替换为你实际的 HAL 库函数)
    Launcher.attach_switch_callbacks(
        [](){ return READ_SW_DELIVER_L;},
        [](){ return READ_SW_DELIVER_R; },
        [](){ return READ_SW_IGNITER; }
    );

    // 任务频率控制
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1);

    // 初始状态设为自检
    Robot.Status.current_state = SYS_CHECKING;
// --- 【新增】自检专用静态变量 ---
    // 记录哪几个开关已经检测过了 (Bitmask)
    static uint8_t check_progress = 0; 
    
    // 记录上一次的按键状态 (用于检测按下瞬间)
    // 初始设为 false (未按下)
    static bool last_sw_L = false;
    static bool last_sw_R = false;
    static bool last_sw_Ign = false;

    // ================== 2. 死循环 ==================
    for (;;)
    {
        // 严格控制 1ms 周期
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        // ---------------- [A] 传感器与输入 (Input) ----------------
        
        // 2. 解析遥控器/视觉指令 -> 存入 Robot.Cmd
        Update_Input(); 

        // 3. 全局离线保护 (优先级最高)
        if (DR16.GetStatus() != DR16_ESTABLISHED) {
            Robot.Status.current_state = SYS_OFFLINE;
        }

        // ---------------- [B] 主状态机逻辑 (Think) ----------------
        
        switch (Robot.Status.current_state)
        {
        case SYS_OFFLINE:
            Launcher.stop(); // 强制断电
            // 恢复条件：遥控器重连
            if (DR16.GetStatus() == DR16_ESTABLISHED) {
                // 重连后，是否要重置自检进度？
                // 如果希望每次断电重连都不用重测，就不要清零 check_progress
                // 如果希望必须重测，则解开下面这行的注释：
                // check_progress = 0; 
                Robot.Status.current_state = SYS_CHECKING;
            }
            break;

        case SYS_CHECKING:
        {
            Launcher.stop(); // 确保电机无力
            
            // 1. 定义检测逻辑 (Lambda函数)
            // 参数: 当前是否按下, 上次是否按下(引用), 对应的掩码位
            auto update_edge = [&](bool is_pressed, bool &last_pressed, uint8_t mask) {
                // 检测上升沿 (从没按 -> 按下)
                if (is_pressed && !last_pressed) {
                    check_progress |= mask; // 标记该位通过
                    // 可以在这里加蜂鸣器：Buzzer.beep(100); 提示检测到
                }
                last_pressed = is_pressed; // 更新历史状态
            };

            // 2. 执行检测 (使用 Driver 提供的接口)
            update_edge(Launcher.get_switch_state_L(),   last_sw_L,   MASK_DELIVER_L);
            update_edge(Launcher.get_switch_state_R(),   last_sw_R,   MASK_DELIVER_R);
            update_edge(Launcher.get_switch_state_Ign(), last_sw_Ign, MASK_IGNITER);

            // 3. 处理跳过逻辑
            if (Robot.Cmd.skip_check) {
                check_progress = MASK_ALL_PASSED; // 强制全满
            }

            // 4. 更新全局标志位
            Robot.Flag.Check.limit_sw_ok = (check_progress == MASK_ALL_PASSED);
            
            // 5. 跳转判断
            if (Robot.Flag.Check.limit_sw_ok) {
                Robot.Status.current_state = SYS_CALIBRATING;
                Launcher.start_calibration();
            }
        }
            break;
        
        case SYS_CALIBRATING:
            // 此状态下，Launcher.run_1ms() 内部正在跑归零逻辑
            // 任务层只需要等待驱动层反馈 "已校准"
            if (Launcher.is_calibrated()) {
                Robot.Status.current_state = SYS_STANDBY;
            }
            break;

        case SYS_STANDBY:
            // --- 待机 / 手动模式 ---
            
            if (Robot.Cmd.manual_override) {
                // 手动微调逻辑
                // 读取当前角度 + 摇杆增量
                float new_igniter_pos = Launcher.get_igniter_angle() + Robot.Cmd.manual_pitch_inc;
                // 将计算结果传给驱动
                Launcher.set_igniter_target(new_igniter_pos);
                
                // 手动模式下，建议把滑块移开，防止卡住
                //Launcher.set_deliver_target(-200.0f); // 缓冲区位置
            }
            
            // 切换到自动模式
            if (Robot.Cmd.auto_mode) {
                Robot.Status.current_state = SYS_AUTO_PREP;
            }
            break;
            
        case SYS_AUTO_PREP:
            // --- 自动发射准备 ---
            // 确保机构归位到待发状态
            Launcher.set_deliver_target(-200.0f); // 回缓冲
            Launcher.set_igniter_target(150.0f);  // 去瞄准默认高度(示例)
            
            // 检查是否到位
            if (Launcher.is_deliver_at_target() && Launcher.is_igniter_at_target()) {
                Robot.Status.current_state = SYS_AUTO_FIRE;
            }
            
            // 随时允许切回手动
            if (!Robot.Cmd.auto_mode) Robot.Status.current_state = SYS_STANDBY;
            break;
            
        case SYS_AUTO_FIRE:
            // --- 自动发射进行中 ---
            
            // 【关键】只有在这个状态下，才运行发射序列子状态机
            Run_Firing_Sequence();
            
            // 随时允许切回手动 (Run_Firing_Sequence 内部也会处理打断复位)
            if (!Robot.Cmd.auto_mode) Robot.Status.current_state = SYS_STANDBY;
            break;
        }

        // ---------------- [C] 执行底层控制 (Act) ----------------
        
        // 只有在非 OFFLINE 且非 CHECKING 时，才允许驱动层输出电流
        // 这是一个双重保险
        if (Robot.Status.current_state != SYS_OFFLINE && 
            Robot.Status.current_state != SYS_CHECKING) 
        {
            // 这是驱动层的“心跳”，每1ms必须调一次
            // 它负责计算 PID、处理归零逻辑、输出电流
            Launcher.run_1ms();
        }
        else
        {
            // 显式停止，防止意外
            Launcher.stop();
        }
        
        // ---------------- [D] 发送 CAN 数据 ----------------
         /*打包数据发送*/
        MotorMsgPack(Tx_Buff, Launch.DeliverMotor[L], Launch.DeliverMotor[R], Launch.IgniterMotor);
		
		xQueueSend(CAN1_TxPort, &Tx_Buff.Id200, 0);
		//这里发现原理图只画了can1的收发器,猜测应该是can1,原代码是发射架的和国镖可能有不同
       // xQueueSend(CAN2_TxPort, &Tx_Buff.Id200, 0);
        //这里说明同步带电机和行程电机都是can2总线，
        //后续进行can2接收测试时，可以和这里互相验证。
    }
}