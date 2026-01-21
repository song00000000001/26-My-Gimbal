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
            #if 0
            // 串级PID: 位置环 -> 速度环
            float target_with_sync = target_deliver_angle + sync_comp[i];
            //target_with_sync=std_lib::constrain(target_with_sync,POS_DELIVER_MIN,POS_DELIVER_MAX);
            pid_deliver_pos[i].Target = target_with_sync;
            //这里加入限幅保护
            //pid_deliver_pos[i].Target=std_lib::constrain(pid_deliver_pos[i].Target,POS_DELIVER_MIN,POS_DELIVER_MAX);
            pid_deliver_pos[i].Current = DeliverMotor[i].getMotorTotalAngle();
            pid_deliver_pos[i].Adjust();
            //速度环的输入为角度环输出
            pid_deliver_spd[i].Target = pid_deliver_pos[i].Out;
            //速度环
            pid_deliver_spd[i].Current = DeliverMotor[i].getMotorSpeed();
            pid_deliver_spd[i].Adjust();
            #else
            //修改同步pid,输出为速度环,防止位置环饱和
            pid_deliver_pos[i].Target=std_lib::constrain(target_deliver_angle,POS_DELIVER_MIN,POS_DELIVER_MAX);
            pid_deliver_pos[i].Current = DeliverMotor[i].getMotorTotalAngle();
            pid_deliver_pos[i].Adjust();
            //速度环的输入为角度环输出加同步补偿
            pid_deliver_spd[i].Target = pid_deliver_pos[i].Out + sync_comp[i];
            //速度环
            pid_deliver_spd[i].Current = DeliverMotor[i].getMotorSpeed();
            pid_deliver_spd[i].Adjust();
            #endif
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
    //清空标志位,重要,修改校准逻辑后记得在这里加复位!不然会造成校准失效。
    Robot.Flag.Status.is_calibrated=0;
	Yawer.Yaw_Init_flag=0;
    is_deliver_homed[0] = false;
    is_deliver_homed[1] = false;
    is_igniter_homed = false;
    // 只有在未校准或强制请求时调用
    for(int i=0; i<2; i++) {
        mode_deliver[i] = MODE_SPEED;
        pid_deliver_spd[i].Target = calibration_speed.deliver_calibration_speed;
        pid_deliver_spd[i].clean_intergral();
        pid_deliver_pos[i].clean_intergral();
    } 
    mode_igniter = MODE_SPEED;
    pid_igniter_spd.Target = calibration_speed.igniter_calibration_speed;
    pid_igniter_spd.clean_intergral();
    pid_igniter_pos.clean_intergral();
}

void Launcher_Driver::check_calibration_logic()
{
    //没有校准则进行校准
    if (!is_deliver_homed[0]) {
        if (SW_DELIVER_L_OFF) {
            LOG_INFO("Deliver Motor L Homing Triggered (Switch Hit)");
            pid_deliver_pos[0].clean_intergral();
			pid_deliver_spd[0].clean_intergral();
            // 1. 消除编码器累积误差 (归零)
			DeliverMotor[0].baseAngle -= (DeliverMotor[0].getMotorTotalAngle()+Debugger.dual_loader_mechanical_error_correction);
			//DeliverMotor[0].baseAngle -= Debugger.dual_loader_mechanical_error_correction; //双滑块机械装配误差校准修正
            // 2. 切换到位置模式
            mode_deliver[0] = MODE_ANGLE;
            // 3. 设定当前位置为回缓冲区
            //pid_deliver_pos[0].Target=POS_BUFFER;
            target_deliver_angle=DELIVER_OFFSET_POS;
            // 4. 标记为已归零            
            is_deliver_homed[0] = true;
        }
    }

    if (!is_deliver_homed[1]) {
        if (SW_DELIVER_R_OFF) {
            LOG_INFO("Deliver Motor R Homing Triggered (Switch Hit)");
            pid_deliver_pos[1].clean_intergral();
			pid_deliver_spd[1].clean_intergral();
            DeliverMotor[1].baseAngle -= DeliverMotor[1].getMotorTotalAngle();
            mode_deliver[1] = MODE_ANGLE;
            //pid_deliver_pos[1].Target=POS_BUFFER;
            target_deliver_angle=DELIVER_OFFSET_POS;  
            is_deliver_homed[1] = true;
        }
    }

    // --- 丝杆归零逻辑 ---
    if (!is_igniter_homed) {
        if (SW_IGNITER_OFF) {
            LOG_INFO("Igniter Motor Homing Triggered (Switch Hit)");
            pid_igniter_pos.clean_intergral();
            pid_igniter_spd.clean_intergral();
            IgniterMotor.baseAngle -= IgniterMotor.getMotorTotalAngle();
            mode_igniter = MODE_ANGLE;
            //pid_igniter_pos.Target=IGNITER_OFFSET_POS;
            target_igniter_angle = IGNITER_OFFSET_POS;
            is_igniter_homed = true;
        }
    }
}

void Launcher_Driver::key_check(){  
    if(SW_DELIVER_L_OFF){
        check_progress |= MASK_DELIVER_L;
    }
    if(SW_DELIVER_R_OFF){
        check_progress |= MASK_DELIVER_R;
    }
    if(SW_IGNITER_OFF){
        check_progress |= MASK_IGNITER;
    }
    if(SW_YAW_L_OFF){
        check_progress |= MASK_YAW_L;
    }
    if(SW_YAW_R_OFF){
        check_progress |= MASK_YAW_R;
    }  
}

void Launcher_Driver::out_all_motor_speed(){
    for(int i=0;i<2;i++){
        DeliverMotor[i].setMotorCurrentOut(pid_deliver_spd[i].Out);
    }
    IgniterMotor.setMotorCurrentOut(pid_igniter_spd.Out);
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

bool Launcher_Driver::is_deliver_sync_ok(float threshold) {
    float diff = fabsf(DeliverMotor[0].getMotorTotalAngle() - DeliverMotor[1].getMotorTotalAngle());
    return (diff < threshold);
}

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

void Launcher_Driver::servo_igniter_lock_f(){
    servo_igniter_lock;// 扳机舵机锁止
}

void Launcher_Driver::servo_igniter_unlock_f(){
    servo_igniter_unlock;// 扳机舵机解锁
}

void Launcher_Driver::servo_transfomer_lock_f(){
    servo_transfomer_lock;// 变压器舵机锁止
}   

void Launcher_Driver::servo_transfomer_unlock_f(){
    servo_transfomer_unlock;// 变压器舵机解锁
}

void Launcher_Driver::servo_loader12_up_f(){
    servo_loader_up1;// 装填舵机1上升
    servo_loader_up2;// 装填舵机2上升
}

void Launcher_Driver::servo_loader12_down_f(){
    servo_loader_down1;// 装填舵机1下降
    servo_loader_down2;// 装填舵机2下降
}

void Launcher_Driver::emergency_override_control(float target_angle){
    target_deliver_angle+=target_angle * 0.1f;
    target_deliver_angle=std_lib::constrain(target_deliver_angle,POS_BOTTOM,POS_DELIVER_MAX);
}