/**
  * @file   launcher_driver.cpp
  */
#include "launcher_driver.h"
#include "robot_config.h" // 包含硬件定义，如 PWM 宏等

/* 常量定义 */
//这里ai的配置和原本的不一样,需要注意
#define DELIVER_HOME_SPEED   (2000.0f) // 归零时的向上速度
#define IGNITER_HOME_SPEED   (1000.0f)
#define DELIVER_OFFSET_POS   (-10.0f)  // 碰到开关后设置的初始坐标
#define IGNITER_OFFSET_POS   (3.0f)

// 构造函数
Launcher_Driver::Launcher_Driver(uint8_t id_l, uint8_t id_r, uint8_t id_ign)
    : DeliverMotor{abstractMotor<Motor_C620>(id_l), abstractMotor<Motor_C620>(id_r)},//这里的警报可以忽略,因为编译通过了
      IgniterMotor(id_ign)
{
    // 电机参数初始化 (极性、减速比)
    DeliverMotor[0].Polarity = POLARITY_DELIVER_L;
    DeliverMotor[1].Polarity = POLARITY_DELIVER_R;
    IgniterMotor.Polarity = POLARITY_IGNITER;

    // 减速比/单位换算
    float deliver_ratio = (2 * PI * 18.62f) / (360 * 51);
    DeliverMotor[0].angle_unit_convert = deliver_ratio;
    DeliverMotor[1].angle_unit_convert = deliver_ratio;
    IgniterMotor.angle_unit_convert = 4.0f / (360.f * 36.f); 

    // 默认为失能状态
    stop();
}

void Launcher_Driver::init()
{
    // PID 参数初始化
    pid_deliver_sync.SetPIDParam(0.5f, 0.0f, 0.0f, 8000, 16000);
    
    for(int i=0; i<2; i++) {
        pid_deliver_spd[i].SetPIDParam(20.0f, 2.0f, 0.0f, 8000, 16380);
        pid_deliver_pos[i].SetPIDParam(800.f, 0.0, 0.0, 1000, 8000);
    }
    
    pid_igniter_spd.SetPIDParam(15.0, 0.0, 0.0, 3000, 16000);
    pid_igniter_pos.SetPIDParam(3000.0, 0.0, 0.0, 3000, 7000);
}

void Launcher_Driver::attach_switch_callbacks(GPIO_PinState (*L)(), GPIO_PinState (*R)(), GPIO_PinState (*Ign)())
{
    read_switch_L = L;
    read_switch_R = R;
    read_switch_Ign = Ign;
}

// ================= 动作接口 =================

void Launcher_Driver::start_calibration()
{
    // 只有在未校准或强制请求时调用
    for(int i=0; i<2; i++) {
        mode_deliver[i] = MODE_HOMING;
        is_deliver_homed[i] = false;
        // 清除积分，防止上次残留
        pid_deliver_spd[i].clean_intergral();
        pid_deliver_pos[i].clean_intergral();
    }
    
    mode_igniter = MODE_HOMING;
    is_igniter_homed = false;
    pid_igniter_spd.clean_intergral();
    pid_igniter_pos.clean_intergral();
}

void Launcher_Driver::stop()
{
    for(int i=0; i<2; i++) mode_deliver[i] = MODE_DISABLE;
    mode_igniter = MODE_DISABLE;
}

void Launcher_Driver::set_deliver_target(float angle)
{
    // 只有在位置模式下才允许设置目标
    if (mode_deliver[0] == MODE_POSITION && mode_deliver[1] == MODE_POSITION) {
        target_deliver_angle = angle; // 软件限位可以在这里加
    }
}

void Launcher_Driver::set_igniter_target(float angle)
{
    if (mode_igniter == MODE_POSITION) {
        target_igniter_angle = angle;
    }
}

void Launcher_Driver::fire_trigger() { servo_igniter_on; }
void Launcher_Driver::fire_reset()   { servo_igniter_off; }

// ================= 核心运行 (run_1ms) =================

void Launcher_Driver::check_calibration_logic()
{
    // --- 滑块归零逻辑 ---
    // 定义局部数组方便遍历 [0]=L, [1]=R
    GPIO_PinState (*read_sw[2])() = {read_switch_L, read_switch_R};
    
    for (int i = 0; i < 2; i++) {
        if (mode_deliver[i] == MODE_HOMING) {
            // 如果碰到开关 (假设低电平触发)
            if (read_sw[i]() == GPIO_PIN_RESET) {
                // 1. 消除编码器累积误差 (归零)
                DeliverMotor[i].baseAngle -= DeliverMotor[i].getMotorTotalAngle();
                
                // 2. 切换到位置模式
                mode_deliver[i] = MODE_POSITION;
                is_deliver_homed[i] = true;
                
                // 3. 设定当前位置为初始目标 (防止跳变)
                target_deliver_angle = DELIVER_OFFSET_POS;
            }
        }
    }

    // --- 丝杆归零逻辑 ---
    if (mode_igniter == MODE_HOMING) {
        if (read_switch_Ign() == GPIO_PIN_RESET) {
            IgniterMotor.baseAngle -= IgniterMotor.getMotorTotalAngle();
            mode_igniter = MODE_POSITION;
            is_igniter_homed = true;
            target_igniter_angle = IGNITER_OFFSET_POS;
        }
    }
}

void Launcher_Driver::run_1ms()
{
    // 1. 处理归零状态转换 (如果在 HOMING 模式)
    check_calibration_logic();

    // 2. 滑块控制循环
    // ------------------------------------------------
    
    // 计算同步误差 (仅在两个都进入位置模式后)
    float sync_comp[2] = {0, 0};
    if (mode_deliver[0] == MODE_POSITION && mode_deliver[1] == MODE_POSITION) {
        // R - L
        float diff = DeliverMotor[1].getMotorTotalAngle() - DeliverMotor[0].getMotorTotalAngle();
        pid_deliver_sync.Target = 0;
        pid_deliver_sync.Current = diff;
        pid_deliver_sync.Adjust();
        // 补偿: R快了就减R加L
        sync_comp[1] = -pid_deliver_sync.Out;
        sync_comp[0] =  pid_deliver_sync.Out;
    } else {
        pid_deliver_sync.clean_intergral();
    }

    for (int i = 0; i < 2; i++) {
        float speed_cmd = 0;

        switch (mode_deliver[i]) {
            case MODE_DISABLE:
                speed_cmd = 0;
                break;
            
            case MODE_HOMING:
                // 纯速度环向上找开关
                speed_cmd = DELIVER_HOME_SPEED; 
                break;
            
            case MODE_POSITION:
                // 串级PID: 位置环 -> 速度环
                pid_deliver_pos[i].Target = target_deliver_angle + sync_comp[i];
                pid_deliver_pos[i].Current = DeliverMotor[i].getMotorTotalAngle();
                pid_deliver_pos[i].Adjust();
                speed_cmd = pid_deliver_pos[i].Out;
                break;
                
            default: speed_cmd = 0; break;
        }

        // 执行速度环
        if (mode_deliver[i] != MODE_DISABLE) {
            pid_deliver_spd[i].Target = speed_cmd;
            pid_deliver_spd[i].Current = DeliverMotor[i].getMotorSpeed();
            pid_deliver_spd[i].Adjust();
            DeliverMotor[i].setMotorCurrentOut(pid_deliver_spd[i].Out);
        } else {
            DeliverMotor[i].setMotorCurrentOut(0);
        }
    }
// 3. 丝杆控制循环 (修正版)
    // ------------------------------------------------
    float ign_spd_cmd = 0;
    switch (mode_igniter) {
        case MODE_DISABLE: 
        case MODE_LOCKED:   // [新增] 锁定模式下，速度目标设为0
            ign_spd_cmd = 0; 
            break;
            
        case MODE_HOMING:  
            ign_spd_cmd = IGNITER_HOME_SPEED; 
            break;
            
        case MODE_POSITION:
            pid_igniter_pos.Target = target_igniter_angle;
            pid_igniter_pos.Current = IgniterMotor.getMotorTotalAngle();
            pid_igniter_pos.Adjust();
            ign_spd_cmd = pid_igniter_pos.Out;
            break;
    }

    // [关键修改] 在这里把 MODE_LOCKED 加入判断条件
    // 只要是 DISABLE 或 LOCKED，都强制输出 0 电流
    if (mode_igniter != MODE_DISABLE && mode_igniter != MODE_LOCKED) {
        pid_igniter_spd.Target = ign_spd_cmd;
        pid_igniter_spd.Current = IgniterMotor.getMotorSpeed();
        pid_igniter_spd.Adjust();
        IgniterMotor.setMotorCurrentOut(pid_igniter_spd.Out);
    } else {
        // 安全保护：切断电流
        IgniterMotor.setMotorCurrentOut(0);
        // 清除积分，防止切换模式瞬间突变
        pid_igniter_spd.clean_intergral();
    }
}

// ================= 状态查询 =================

bool Launcher_Driver::is_calibrated() {
    return is_deliver_homed[0] && is_deliver_homed[1] && is_igniter_homed;
}

bool Launcher_Driver::is_deliver_at_target() {
    // 简单判断误差
    return (abs(DeliverMotor[0].getMotorTotalAngle() - target_deliver_angle) < 5.0f);
}

bool Launcher_Driver::is_igniter_at_target() {
    return (abs(IgniterMotor.getMotorTotalAngle() - target_igniter_angle) < 2.0f);
}

bool Launcher_Driver::get_switch_state_L() { return (read_switch_L() == GPIO_PIN_RESET); }
bool Launcher_Driver::get_switch_state_R() { return (read_switch_R() == GPIO_PIN_RESET); }
bool Launcher_Driver::get_switch_state_Ign() { return (read_switch_Ign() == GPIO_PIN_RESET); }

float Launcher_Driver::get_igniter_angle() { return IgniterMotor.getMotorTotalAngle(); }