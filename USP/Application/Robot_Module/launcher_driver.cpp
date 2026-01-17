#include "launcher_driver.h"
#include "robot_config.h"
#include "global_data.h"

// 构造函数
Launcher_Driver::Launcher_Driver(uint8_t id_l, uint8_t id_r, uint8_t id_ign)
    : DeliverMotor{abstractMotor<Motor_C620>(id_l), abstractMotor<Motor_C620>(id_r)},//这里的警报可以忽略,因为编译通过了
      IgniterMotor(id_ign)
{
    stop_all_motor();
    //初始化舵机位置
    servo_loader_up1;
    servo_loader_up2;
    servo_transfomer_lock;
    servo_igniter_lock;

    // 电机参数初始化 (极性、减速比)
    DeliverMotor[0].Polarity = POLARITY_DELIVER_L;
    DeliverMotor[1].Polarity = POLARITY_DELIVER_R;
    IgniterMotor.Polarity = POLARITY_IGNITER;
    // 校准状态初始化
    is_deliver_homed[0] = false;
    is_deliver_homed[1] = false;
    is_igniter_homed = false;
    // 减速比/单位换算
    float deliver_ratio = (2 * PI * 18.62f) / (360 * 51);
    DeliverMotor[0].angle_unit_convert = deliver_ratio;
    DeliverMotor[1].angle_unit_convert = deliver_ratio;
    IgniterMotor.angle_unit_convert = 4.0f / (360.f * 19.0f); 


    // 自检开关检测进度初始化
    check_progress=0; 
    // 初始模式均为速度环
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
            float target_with_sync = target_deliver_angle + sync_comp[i];
            target_with_sync=std_lib::constrain(target_with_sync,POS_DELIVER_MIN,POS_DELIVER_MAX);
            pid_deliver_pos[i].Target = target_with_sync;
            //这里加入限幅保护
            pid_deliver_pos[i].Target=std_lib::constrain(pid_deliver_pos[i].Target,POS_DELIVER_MIN,POS_DELIVER_MAX);
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
    if(mode_igniter==MODE_ANGLE){
        // 串级PID: 位置环 -> 速度环
        target_igniter_angle=std_lib::constrain(target_igniter_angle,IGNITER_MIN_POS,IGNITER_MAX_POS);
        pid_igniter_pos.Target = target_igniter_angle;
        //这里加入限幅保护
        pid_igniter_pos.Target=std_lib::constrain(pid_igniter_pos.Target, IGNITER_MIN_POS, IGNITER_MAX_POS);
        pid_igniter_pos.Current = IgniterMotor.getMotorTotalAngle();
        pid_igniter_pos.Adjust();
        //速度环的输入为角度环输出
        pid_igniter_spd.Target = pid_igniter_pos.Out;
        //速度环
        pid_igniter_spd.Current = IgniterMotor.getMotorSpeed();
        pid_igniter_spd.Adjust();
    }
    else if(mode_igniter==MODE_SPEED){
        pid_igniter_spd.Current = IgniterMotor.getMotorSpeed();
        pid_igniter_spd.Adjust();
    }
    else{
        IgniterMotor.setMotorCurrentOut(0);
        pid_igniter_pos.clean_intergral();
        pid_igniter_spd.clean_intergral();
    }
    //校准后,限位开关意外触发记录
    #if 0
    //由于限位开关延迟问题，容易误触发，这里先注释掉
    if(is_calibrated()){
        if(SW_DELIVER_L_OFF) {
            LOG_WARN("Left Deliver Limit Switch Triggered when calibrated");
        }
        if(SW_DELIVER_R_OFF){
            LOG_WARN("Right Deliver Limit Switch Triggered when calibrated");
        }
        if(SW_IGNITER_OFF){
            LOG_WARN("Left Deliver Limit Switch Triggered when calibrated");
        }
    }     
    #endif
}
/*todo
song
在can接收那里写一个电机失联判断
    if (DeliverMotor[0].)) {
        LOG_ERROR("Deliver Motor 0 Connection LOST!");
    }    
*/
void Launcher_Driver::start_calibration()
{
    LOG_INFO("Launcher Calibration Started");
    //清空标志位,以便多次校准
	Yawer.Yaw_Init_flag=0;
    is_deliver_homed[0] = false;
    is_deliver_homed[1] = false;
    is_igniter_homed = false;
    // 只有在未校准或强制请求时调用
    for(int i=0; i<2; i++) {
        mode_deliver[i] = MODE_SPEED;
        pid_deliver_spd[i].Target = calibration_speed.deliver_calibration_speed;//这里的速度在check逻辑里设置
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
    if (!is_deliver_homed[0]) {
        if (SW_DELIVER_L_OFF) {
            LOG_INFO("Deliver Motor L Homing Triggered (Switch Hit)");
            pid_deliver_pos[0].clean_intergral();
			pid_deliver_spd[0].clean_intergral();
            // 1. 消除编码器累积误差 (归零)
			DeliverMotor[0].baseAngle -= DeliverMotor[0].getMotorTotalAngle();
			
            pid_deliver_pos[0].Target=DELIVER_OFFSET_POS;
            
            // 2. 切换到位置模式
            mode_deliver[0] = MODE_ANGLE;
            is_deliver_homed[0] = true;
            
            // 3. 设定当前位置为初始目标
            #if 0
            target_deliver_angle = DELIVER_OFFSET_POS;
            #else
            //修改为回缓冲区
            target_deliver_angle=POS_BUFFER;   // 回缓冲
            #endif
        }
    }
    //为了方便stanby状态下失能电机（速度目标为0）后，进入发射状态时能方便的恢复速度，这里加入速度设置（所以前面start里的速度设置可以删了。）
    else{
        pid_deliver_spd[0].Target = calibration_speed.deliver_calibration_speed;  
    }

    if (!is_deliver_homed[1]) {
        if (SW_DELIVER_R_OFF) {
            LOG_INFO("Deliver Motor R Homing Triggered (Switch Hit)");
            pid_deliver_pos[1].clean_intergral();
			pid_deliver_spd[1].clean_intergral();
			// 1. 消除编码器累积误差 (归零)
            DeliverMotor[1].baseAngle -= DeliverMotor[1].getMotorTotalAngle();
            pid_deliver_pos[1].Target=DELIVER_OFFSET_POS;  
            
            // 2. 切换到位置模式
            mode_deliver[1] = MODE_ANGLE;
            is_deliver_homed[1] = true;
            
            // 3. 设定当前位置为初始目标
            #if 0
            target_deliver_angle = DELIVER_OFFSET_POS;
            #else
            target_deliver_angle=POS_BUFFER;   // 回缓冲
            #endif
        }
    }
    else{
        pid_deliver_spd[1].Target = calibration_speed.deliver_calibration_speed;  
    }

    // --- 丝杆归零逻辑 ---
    if (!is_igniter_homed) {
        if (SW_IGNITER_OFF) {
            LOG_INFO("Igniter Motor Homing Triggered (Switch Hit)");
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
    if(SW_DELIVER_L_OFF){
        check_progress |= MASK_DELIVER_L;
        LOG_INFO("Switch Deliver L Checked,all switch check status:%s",check_progress);   
    }
    if(SW_DELIVER_R_OFF){
        check_progress |= MASK_DELIVER_R;
        LOG_INFO("Switch Deliver R Checked,all switch check status:%s",check_progress);
    }
    if(SW_IGNITER_OFF){
        check_progress |= MASK_IGNITER;
        LOG_INFO("Switch Igniter Checked,all switch check status:%s",check_progress);
    }
    if(SW_YAW_L_OFF){
        check_progress |= MASK_YAW_L;
        LOG_INFO("Switch Yaw L Checked,all switch check status:%s",check_progress);
    }
    if(SW_YAW_R_OFF){
        check_progress |= MASK_YAW_R;
        LOG_INFO("Switch Yaw R Checked,all switch check status:%s",check_progress);
    }  
}

void Launcher_Driver::out_all_motor_speed(){
    for(int i=0;i<2;i++){
        DeliverMotor[i].setMotorCurrentOut(pid_deliver_spd[i].Out);
    }
    IgniterMotor.setMotorCurrentOut(pid_igniter_spd.Out);
}


/*todo
   song
   考虑将yaw合并到发射类中。
*/


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
    servo_igniter_lock;
}

// ================= 状态查询 =================
bool Launcher_Driver::is_calibrated() {
    return is_deliver_homed[0] && is_deliver_homed[1] && is_igniter_homed;
}

// 状态查询细化
bool Launcher_Driver::is_deliver_L_calibrated() {
    return is_deliver_homed[0];
}

bool Launcher_Driver::is_deliver_R_calibrated() {
    return is_deliver_homed[1];
}

bool Launcher_Driver::is_igniter_calibrated() {
    return is_igniter_homed;
}

bool Launcher_Driver::is_deliver_at_target(float threshold) {
    float err=abs(DeliverMotor[0].getMotorTotalAngle() - target_deliver_angle);
    return (err < threshold);
}

bool Launcher_Driver::is_igniter_at_target(float threshold) {
    float err=abs(IgniterMotor.getMotorTotalAngle() - target_igniter_angle);
    return (err < threshold);
}

/*
//todo: 视觉瞄准到位触发
Robot.Cmd.fire_command=true;
if (Robot.Cmd.fire_command&&is_deliver_at_target(5)) {
    fire_state = FIRE_PULL_LOAD;
}
*/

// 发射状态机中校准滑块电机
void Launcher_Driver::start_deliver_calibration()
{
    LOG_INFO("Deliver Calibration Started in Firing Sequence");
    //调整滑块电机状态
    for(int i=0;i<2;i++){
        //清空标志位
        is_deliver_homed[i] = false;
        //设置速度环模式
        mode_deliver[i] = MODE_SPEED;
        //设置校准速度
        pid_deliver_spd[i].Target = calibration_speed.deliver_calibration_speed;
        // 清除积分，防止上次残留
        pid_deliver_spd[i].clean_intergral();
        pid_deliver_pos[i].clean_intergral();
    }
}


void Launcher_Driver::servo_pwm_test_lock_up(){
    servo_transfomer_lock;
    servo_loader_up1;
    servo_loader_up2;
    servo_igniter_lock;
}

void Launcher_Driver::servo_pwm_test_unlock_down(){
    servo_transfomer_unlock;
	servo_loader_down1;
    servo_loader_down2;
    servo_igniter_unlock;
}

void Launcher_Driver::loader_servo_1_ctrl(uint16_t ccr){
    if(servo_ccr.loader1_ccr_down<servo_ccr.loader1_ccr_up)
        ccr=std_lib::constrain(ccr, servo_ccr.loader1_ccr_down, servo_ccr.loader1_ccr_up);
    else
        ccr=std_lib::constrain(ccr, servo_ccr.loader1_ccr_up, servo_ccr.loader1_ccr_down);
    
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1,ccr);  // 装调舵机左，下降
}

void Launcher_Driver::loader_servo_2_ctrl(uint16_t ccr){
    if(servo_ccr.loader2_ccr_down<servo_ccr.loader2_ccr_up)
        ccr=std_lib::constrain(ccr, servo_ccr.loader2_ccr_down, servo_ccr.loader2_ccr_up);
    else
        ccr=std_lib::constrain(ccr, servo_ccr.loader2_ccr_up, servo_ccr.loader2_ccr_down);
    
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, ccr);  // 装填舵机右，上升
}
