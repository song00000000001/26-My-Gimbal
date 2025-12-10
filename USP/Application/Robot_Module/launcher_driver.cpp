#include "launcher_driver.h"
#include "robot_config.h"
#include "global_data.h"

//定义舵机测试结构体，方便调节测试舵机行程
typedef struct 
{
    uint16_t igniter_ccr_unlock;
    uint16_t igniter_ccr_lock;
    uint16_t loader1_ccr_up;
    uint16_t loader1_ccr_down;
    uint16_t loader2_ccr_up;
    uint16_t loader2_ccr_down;
    uint16_t transfomer_ccr_lock;
    uint16_t transfomer_ccr_unlock;
}servo_ccr_debug;

servo_ccr_debug servo_ccr={
    170,    //igniter_ccr_unlock
    270,    //igniter_ccr_lock
    53,     //loader1_ccr_up
    100,    //loader1_ccr_down
    288,    //loader2_ccr_up
    360,    //loader2_ccr_down
    126,    //transfomer_ccr_lock
    170     //transfomer_ccr_unlock
};

/* ==舵机宏== */

#define servo_igniter_unlock    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1,servo_ccr.igniter_ccr_unlock ) // 扳机舵机解锁      ,120卡住,170ok
#define servo_igniter_lock      __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1,servo_ccr.igniter_ccr_lock ) // 扳机舵机锁止

/*todo
查出引脚填写在下面。
定义装填舵机和转移舵机的宏
*/
//装填舵机即升降机左右的舵机，上为升，下为下降，下降即装填飞镖，上升即清空发射区
#define servo_loader_up1     __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, servo_ccr.loader1_ccr_up)   // 装填舵机左，上升
#define servo_loader_down1   __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, servo_ccr.loader1_ccr_down)  // 装调舵机左，下降

#define servo_loader_up2     __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, servo_ccr.loader2_ccr_up)  // 装填舵机右，上升
#define servo_loader_down2   __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, servo_ccr.loader2_ccr_down)  // 装填舵机右，下降
//转移舵机即动作舱储存区的卡镖舵机，负责将新镖从储存区转移到发射区
#define servo_transfomer_lock     __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, servo_ccr.transfomer_ccr_lock)  // 卡镖舵机维持卡镖
#define servo_transfomer_unlock   __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, servo_ccr.transfomer_ccr_unlock)  // 卡镖舵机松开卡镖

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
    pid_deliver_sync.SetPIDParam(-0.4f, 0.0f, 0.0f, 8000, 16000);
    
    for(int i=0; i<2; i++) {
        pid_deliver_spd[i].SetPIDParam(20.0f, 2.0f, 0.0f, 8000, 16380);
        pid_deliver_pos[i].SetPIDParam(800.f, 0.0, 0.0, 1000, 8000);
    }
    
    pid_igniter_spd.SetPIDParam(15.0, 0.0, 0.0, 3000, 12000);
    pid_igniter_pos.SetPIDParam(3000.0, 0.0, 0.0, 3000, 6000);

    // 自检开关检测进度
    check_progress=0; 
    // 初始化堵转计时器
    stall_timer_deliver[0] = 0; 
    stall_timer_deliver[1] = 0;
    stall_timer_igniter = 0;
    mode_deliver[0] = MODE_SPEED;
    mode_deliver[1] = MODE_SPEED;
    mode_igniter = MODE_SPEED;
}

// ================= 动作接口 =================
void Launcher_Driver::adjust()
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
        pid_deliver_spd[i].Target = calibration_speed.deliver_calibration_speed;
        // 清除积分，防止上次残留
        pid_deliver_spd[i].clean_intergral();
        pid_deliver_pos[i].clean_intergral();
    } 
    mode_igniter = MODE_SPEED;
    pid_igniter_spd.Target = calibration_speed.igniter_calibration_speed;
     // 清除积分，防止上次残留
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
            
            // 3. 设定当前位置为初始目标
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
            
            // 3. 设定当前位置为初始目标
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


/*发射状态机*/

void Launcher_Driver::Run_Firing_Sequence()
{
    
    static uint32_t state_timer = 0;

    switch (fire_state)
    {
        case FIRE_IDLE:
            // 确保滑块在缓冲区
            target_deliver_angle=(POS_BUFFER);
            fire_lock(); // 锁止扳机舵机
            //todo: 视觉瞄准到位触发
            Robot.Cmd.fire_command=true;
            if (Robot.Cmd.fire_command&&is_deliver_at_target()) {
                fire_state = FIRE_PULL_LOAD;
            }
            break;

        case FIRE_PULL_LOAD:
            //下拉滑块到装填位置
            target_deliver_angle=(POS_WAITLOAD);
            if (is_deliver_at_target()) {
                state_timer = xTaskGetTickCount();
                fire_state = FIRE_WAITLOAD;
            }
            break;

        case FIRE_WAITLOAD:
            //todo,调用装填舵机,使升降机下放
            //等装填完毕
            if ((xTaskGetTickCount() - state_timer) > 500) {
                fire_state = FIRE_PULL_BOTTOM;
            }
            break;

        case FIRE_PULL_BOTTOM:
            //滑块全速下拉到底
            target_deliver_angle=(POS_BOTTOM);
            if (is_deliver_at_target()) {
                state_timer = xTaskGetTickCount();
                fire_state = FIRE_LATCHING;
            }
            break;

        case FIRE_LATCHING:
            //装填舵机动作使升降机归位
            if ((xTaskGetTickCount() - state_timer) > 500) {
                fire_state = FIRE_RETURNING;
            }
            break;

        case FIRE_RETURNING:
            // 滑块回缓冲区
            target_deliver_angle=(POS_BUFFER);
            if (is_deliver_at_target()) {
                fire_state = FIRE_TRANSFORE;
                state_timer = xTaskGetTickCount();
            }
            break;
            
        case FIRE_TRANSFORE:
            //卡镖舵机松开，转移新镖到发射区
            servo_transfomer_unlock;
            if ((xTaskGetTickCount() - state_timer) > 250) {
                state_timer = xTaskGetTickCount();
                fire_state = FIRE_TRANSFORE_BACK;
            }

            break;
            
        case FIRE_TRANSFORE_BACK:
            //卡镖舵机回正，卡住新镖
            servo_transfomer_lock;
            if ((xTaskGetTickCount() - state_timer) > 250) {
                state_timer = xTaskGetTickCount();
                fire_state = FIRE_SHOOTING;
            }
            break;

        case FIRE_SHOOTING:
            servo_igniter_unlock; // 解锁扳机舵机，发射
            // 等待发射完成
            if ((xTaskGetTickCount() - state_timer) > 3000) {
                Robot.Status.dart_count++; // 计数+1
                // 判断是否继续连发
                // if (Robot.Status.dart_count < xxx) ...
                fire_state = FIRE_IDLE;
            }
            break;
        
        // 异常打断：如果突然切出手制动，重置状态机
        if (!Robot.Cmd.auto_mode) {
            fire_state = FIRE_IDLE;
        }
    }
}

// --- 堵转检测 (无电流计版) ---
// limit_output: 当 PID 输出(即目标电流)超过此值
// time_ms: 持续时间
bool Launcher_Driver::check_deliver_stall(float limit_output,float threhold_rpm, uint32_t time_ms)
{
    uint32_t now = xTaskGetTickCount();
    bool is_stalled = false;
    bool stalled_check=(abs(pid_deliver_spd[0].Out)>limit_output&&abs(DeliverMotor[0].getMotorSpeed())<threhold_rpm);
    stalled_check|=(abs(pid_deliver_spd[1].Out)>limit_output&&abs(DeliverMotor[1].getMotorSpeed())<threhold_rpm);
    if(stalled_check) {
        if (stall_timer_deliver[0] == 0) stall_timer_deliver[0] = now;
        else if (now - stall_timer_deliver[0] > time_ms) is_stalled = true;
    } 
    else {
        stall_timer_deliver[0] = 0;
    }
    return is_stalled;
}

bool Launcher_Driver::check_igniter_stall(float limit_output, float threhold_rpm, uint32_t time_ms)
{ 
    uint32_t now = xTaskGetTickCount();
    bool is_stalled = false;
    if (abs(pid_igniter_spd.Out) > limit_output && abs(IgniterMotor.getMotorSpeed()) < threhold_rpm) {
        if (stall_timer_igniter == 0) stall_timer_igniter = now;
        else if (now - stall_timer_igniter > time_ms) is_stalled = true;
    } 
    else {
        stall_timer_igniter = 0;
    }
    
    return is_stalled;
}

