#include "internal.h"
#include "global_data.h"
#include "launcher_driver.h"
#include "robot_config.h"

#define POS_BUFFER -20
#define POS_BOTTOM -620

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

    switch (fire_state)
    {
    case FIRE_IDLE:
        // 确保滑块在缓冲区
        Launcher.set_deliver_target(POS_BUFFER);
        Launcher.fire_lock(); // 关舵机
        
        // 触发条件：收到指令 且 视觉瞄准到位(可选)
        // 这里假设 Cmd.fire_command 在自动模式下由视觉改写，或者简单的连发逻辑
		Robot.Cmd.fire_command=true;
        if (Robot.Cmd.fire_command) {
            fire_state = FIRE_PULLING;
        }
        break;

    case FIRE_PULLING:
        
        // 2. 滑块全速下拉
        Launcher.fire_lock();//锁止扳机,准备扣上滑台
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
        if ((xTaskGetTickCount() - state_timer) > 2000) {
            Robot.Status.dart_count++; // 计数+1
            fire_state = FIRE_COOLDOWN;
            state_timer = xTaskGetTickCount();
        }
        break;

    case FIRE_COOLDOWN:
        Launcher.fire_lock(); 
        
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
 * @brief 发射主控任务
 */
void LaunchCtrl(void *arg)
{
    //can发送的包
    Motor_CAN_COB Tx_Buff;
 
    // 初始状态设为自检
    Robot.Status.yaw_control_state=disable_motor; //yaw轴失能
    Robot.Status.current_state = SYS_CHECKING;
	Robot.Flag.Check.limit_sw_ok=false;
    
    // 任务频率控制
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1);
    for (;;)
    {
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
        {
            stop_all_motor();
            // 恢复条件：遥控器重连
            if (DR16.GetStatus() == DR16_ESTABLISHED) {
                Robot.Status.current_state = SYS_CHECKING;
            }
        }
        break;
        case SYS_CHECKING:
        {
            stop_all_motor(); 
            //按键自检逻辑
            key_check();
            // 5个按键手动检查全部通过则进入校准状态,后续可以加入电机检查
            if (Robot.Flag.Check.limit_sw_ok) {
                Robot.Status.current_state = SYS_CALIBRATING;
                Launcher.start_calibration();
            }
        }
        break;
        
        case SYS_CALIBRATING:
            // 此状态下，Launcher.run_1ms() 内部正在跑归零逻辑，任务层只需要等待驱动层反馈 "已校准"
            //yaw控制考虑到是并行的，主状态机和子状态机采用状态位判断
            Robot.Status.yaw_control_state = YAW_CALIBRATING;

            //校准完毕跳转待机状态
            if (Yawer.is_Yaw_Init() == 1&&Launcher.is_calibrated()) {
                Robot.Status.current_state = SYS_STANDBY;
            }
            break;

        case SYS_STANDBY:
            // --- 待机 / 手动模式 ---
            Launcher.set_deliver_target(POS_BUFFER); // 回缓冲
            if (Robot.Cmd.manual_override) {
                // 手动微调逻辑
                // 读取当前角度 + 摇杆增量
                //float new_igniter_pos = Launcher.get_igniter_angle() + Robot.Cmd.manual_pitch_inc;
                // 将计算结果传给驱动
                //Launcher.set_igniter_target(new_igniter_pos);
                
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
			Launcher.set_deliver_target(POS_BUFFER); // 回缓冲
            //Launcher.set_igniter_target(150.0f);  // 去瞄准默认高度(示例)
            
            // 检查是否到位
            if (Launcher.is_deliver_at_target() )//&& Launcher.is_igniter_at_target()) 
            {  
                Robot.Status.current_state = SYS_AUTO_FIRE;
            }
            
            // 随时允许切回手动
            if (!Robot.Cmd.auto_mode) Robot.Status.current_state = SYS_STANDBY;
            break;
            
        case SYS_AUTO_FIRE:
            Run_Firing_Sequence();
            // 随时允许切回手动 (Run_Firing_Sequence 内部也会处理打断复位)
            if (!Robot.Cmd.auto_mode) Robot.Status.current_state = SYS_STANDBY;
            break;
        }
        // ---------------- [C] 执行底层控制 (Act) ----------------
        
        // 只有在非 OFFLINE 且非 CHECKING 时，才允许驱动层输出电流
        // 这是一个双重保险
        if (Robot.Status.current_state != SYS_OFFLINE && 
            Robot.Status.current_state != SYS_CHECKING&&DR16.GetS1()!=SW_UP) 
        {
            // 驱动层，负责计算 PID、处理归零逻辑、输出电流
            Launcher.run_1ms();
        }
        else
        {
            // 显式停止，防止意外
            stop_all_motor();
        }
        
        // ---------------- [D] 发送 CAN 数据 ----------------
        /*打包数据发送*/
        MotorMsgPack(Tx_Buff, Launcher.DeliverMotor[L], Launcher.DeliverMotor[R], Launcher.IgniterMotor);
		xQueueSend(CAN1_TxPort, &Tx_Buff.Id200, 0);
    }
}