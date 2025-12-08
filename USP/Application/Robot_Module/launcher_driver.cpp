/**
  * @file   launcher_driver.cpp
  */
#include "launcher_driver.h"
#include "robot_config.h" // 包含硬件定义，如 PWM 宏等



// 构造函数
Launcher_Driver::Launcher_Driver(uint8_t id_l, uint8_t id_r, uint8_t id_ign)
    : DeliverMotor{abstractMotor<Motor_C620>(id_l), abstractMotor<Motor_C620>(id_r)},//这里的警报可以忽略,因为编译通过了
      IgniterMotor(id_ign)
{
    stop_all_motor();

    // 电机参数初始化 (极性、减速比)
    DeliverMotor[0].Polarity = POLARITY_DELIVER_L;
    DeliverMotor[1].Polarity = POLARITY_DELIVER_R;
    IgniterMotor.Polarity = POLARITY_IGNITER;
    is_deliver_homed[0] = false;
    is_deliver_homed[1] = false;
    is_igniter_homed = false;
    // 减速比/单位换算
    float deliver_ratio = (2 * PI * 18.62f) / (360 * 51);
    DeliverMotor[0].angle_unit_convert = deliver_ratio;
    DeliverMotor[1].angle_unit_convert = deliver_ratio;
    IgniterMotor.angle_unit_convert = 4.0f / (360.f * 36.f); 

    // PID 参数初始化
    pid_deliver_sync.SetPIDParam(0.5f, 0.0f, 0.0f, 8000, 16000);
    
    for(int i=0; i<2; i++) {
        pid_deliver_spd[i].SetPIDParam(20.0f, 2.0f, 0.0f, 8000, 16380);
        pid_deliver_pos[i].SetPIDParam(800.f, 0.0, 0.0, 1000, DELIVER_MAX_SPEED);
    }
    
    pid_igniter_spd.SetPIDParam(15.0, 0.0, 0.0, 3000, 12000);
    pid_igniter_pos.SetPIDParam(3000.0, 0.0, 0.0, 3000, IGNITER_MAX_SPEED);

    // 自检开关检测进度
    check_progress=0; 
}

// ================= 动作接口 =================
void Launcher_Driver::run_1ms()
{
    // 计算同步误差 (仅在两个都进入位置模式后)
    float sync_comp[2] = {0, 0};
    if (mode_deliver[0] == MODE_ANGLE && mode_deliver[1] == MODE_ANGLE) {
        // R - L
        float diff = DeliverMotor[1].getMotorTotalAngle() - DeliverMotor[0].getMotorTotalAngle();
        pid_deliver_sync.Target = 0;
        pid_deliver_sync.Current = diff;
        pid_deliver_sync.Adjust();
        // 补偿: R快了就减R加L
        sync_comp[1] = -pid_deliver_sync.Out;
        sync_comp[0] =  pid_deliver_sync.Out;
    } 
    else {
        pid_deliver_sync.clean_intergral();
    }

    for (int i = 0; i < 2; i++) {
        //按需执行角度环
        if(mode_deliver[i]==MODE_ANGLE){
            // 串级PID: 位置环 -> 速度环
            pid_deliver_pos[i].Target = target_deliver_angle + sync_comp[i];
            pid_deliver_pos[i].Current = DeliverMotor[i].getMotorTotalAngle();
            pid_deliver_pos[i].Adjust();
            //速度环的输入为角度环输出
            pid_deliver_spd[i].Target = pid_deliver_pos[i].Out;
            //速度环
            pid_deliver_spd[i].Current = DeliverMotor[i].getMotorSpeed();
            pid_deliver_spd[i].Adjust();
        }
        else if(mode_deliver[i]==MODE_SPEED){
            pid_deliver_spd[i].Current = DeliverMotor[i].getMotorSpeed();
            pid_deliver_spd[i].Adjust();
        }
        else{
            DeliverMotor[i].setMotorCurrentOut(0);
            pid_deliver_spd[i].clean_intergral();
            pid_deliver_pos[i].clean_intergral();
        }
    }
    if(mode_deliver[0]==MODE_ANGLE){
        // 串级PID: 位置环 -> 速度环
        pid_igniter_pos.Target = target_igniter_angle;
        pid_igniter_pos.Current = IgniterMotor.getMotorTotalAngle();
        pid_igniter_pos.Adjust();
        pid_igniter_spd.Target = pid_igniter_pos.Out;
        //速度环的输入为角度环输出
        pid_igniter_spd.Target = pid_deliver_pos[0].Out;
        //速度环
        pid_igniter_spd.Current = IgniterMotor.getMotorSpeed();
        pid_igniter_spd.Adjust();
    }
    else if(mode_deliver[0]==MODE_SPEED){
        pid_igniter_spd.Current = IgniterMotor.getMotorSpeed();
        pid_igniter_spd.Adjust();
    }
    else{
        IgniterMotor.setMotorCurrentOut(0);
        pid_igniter_pos.clean_intergral();
        pid_igniter_spd.clean_intergral();
    }

}

void Launcher_Driver::start_calibration()
{
    // 只有在未校准或强制请求时调用
    for(int i=0; i<2; i++) {
        mode_deliver[i] = MODE_SPEED;
        pid_deliver_spd[i].Target = DELIVER_HOME_SPEED;
        // 清除积分，防止上次残留
        pid_deliver_spd[i].clean_intergral();
        pid_deliver_pos[i].clean_intergral();
    } 
    mode_igniter = MODE_SPEED;
    pid_igniter_spd.Target = IGNITER_HOME_SPEED;
    pid_igniter_spd.clean_intergral();
    pid_igniter_pos.clean_intergral();
}

void Launcher_Driver::check_calibration_logic()
{
    // --- 滑块归零逻辑 ---
    // 定义局部数组方便遍历 [0]=L, [1]=R
    if (!is_deliver_homed[0]) {
        // 如果碰到开关 (假设低电平触发)
        if (SW_DELIVER_L_OFF) {
			pid_deliver_spd[0].clean_intergral();
			 // 1. 消除编码器累积误差 (归零)
			DeliverMotor[0].baseAngle -= DeliverMotor[0].getMotorTotalAngle();
			
            pid_deliver_pos[0].Target=DELIVER_OFFSET_POS;
            
            // 2. 切换到位置模式
            mode_deliver[0] = MODE_ANGLE;
            is_deliver_homed[0] = true;
            
            // 3. 设定当前位置为初始目标 (防止跳变)
            target_deliver_angle = DELIVER_OFFSET_POS;
        }
    }

    if (!is_deliver_homed[0]) {
        // 如果碰到开关 (假设低电平触发)
        if (SW_DELIVER_R_OFF) {
			pid_deliver_spd[1].clean_intergral();
			// 1. 消除编码器累积误差 (归零)
            DeliverMotor[1].baseAngle -= DeliverMotor[1].getMotorTotalAngle();
            pid_deliver_pos[1].Target=DELIVER_OFFSET_POS;  
            
            // 2. 切换到位置模式
            mode_deliver[1] = MODE_ANGLE;
            is_deliver_homed[1] = true;
            
            // 3. 设定当前位置为初始目标 (防止跳变)
            target_deliver_angle = DELIVER_OFFSET_POS;
        }
    }

    // --- 丝杆归零逻辑 ---
    if (!is_igniter_homed) {
        if (SW_IGNITER_OFF) {
            pid_igniter_pos.clean_intergral();
            pid_igniter_spd.clean_intergral();
            IgniterMotor.baseAngle -= IgniterMotor.getMotorTotalAngle();
            mode_igniter = MODE_ANGLE;
            is_igniter_homed = true;
            target_igniter_angle = IGNITER_OFFSET_POS;
        }
    }
}

void Launcher_Driver::key_check(){  

    if(SW_YAW_L_OFF){
        check_progress |= MASK_YAW_L;
    }
    if(SW_YAW_R_OFF){
        check_progress |= MASK_YAW_R;
    }
    if(SW_DELIVER_L_OFF){
        check_progress |= MASK_DELIVER_L;
    }
    if(SW_DELIVER_R_OFF){
        check_progress |= MASK_DELIVER_R;
    }
    if(SW_IGNITER_OFF){
        check_progress |= MASK_IGNITER;
    }
    
}

//解锁扳机舵机
void Launcher_Driver::fire_unlock() { servo_igniter_unlock; }
//锁定扳机舵机
void Launcher_Driver::fire_lock()   { servo_igniter_lock; }

void Launcher_Driver::out_all_motor_speed(){
    for(int i=0;i<2;i++){
        DeliverMotor[i].setMotorCurrentOut(pid_deliver_spd[i].Out);
    }
    IgniterMotor.setMotorCurrentOut(pid_igniter_spd.Out);
}

void Launcher_Driver::stop_yaw_motor(){
    // Yawer.disable();
   /*todo
   song
   将yaw合并到发射类中,考虑下合并事宜后再操作,现在检查电机状态问题.
   */
}

void Launcher_Driver::stop_deliver_motor(){
    for(int i=0;i<2;i++){
        pid_deliver_spd[i].clean_intergral();
        pid_deliver_pos[i].clean_intergral();
        DeliverMotor[i].setMotorCurrentOut(0);
    }
}

void Launcher_Driver::stop_igniter_motor(){
    pid_igniter_pos.clean_intergral();
    pid_igniter_spd.clean_intergral();
    IgniterMotor.setMotorCurrentOut(0);
}

void Launcher_Driver::stop_all_motor(){
    stop_igniter_motor();
    stop_deliver_motor();
    stop_yaw_motor();

    fire_lock();
}

// ================= 状态查询 =================
bool Launcher_Driver::is_calibrated() {
    return is_deliver_homed[0] && is_deliver_homed[1] && is_igniter_homed;
}

bool Launcher_Driver::is_deliver_at_target() {
    uint16_t err=abs(DeliverMotor[0].getMotorTotalAngle() - target_deliver_angle);
    return (err < 5.0f);
}

bool Launcher_Driver::is_igniter_at_target() {
    uint16_t err=abs(IgniterMotor.getMotorTotalAngle() - target_igniter_angle);
    return (err < 5.0f);
}