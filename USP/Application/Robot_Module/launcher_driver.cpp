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
    /*
    代码逻辑解释：通用溢出处理：
    我没有使用复杂的 if(i==1) 或硬编码 2*Sync，而是采用了一个更通用的逻辑：计算溢出量（Overflow）。
    比如目标是 8500，限幅 8000，溢出量就是 +500。比如目标是 -8200，限幅 -8000，溢出量就是 -200。
    维持相对差：假设左边算出来是 8500（溢出+500），右边是 7500（溢出 0）。
    我们取最大溢出量 +500。左边：$8500 - 500 = 8000$。右边：$7500 - 500 = 7000$。
    结果：左边顶着限幅跑，右边自动降速，两者的差值依然是 1000，实现了“削峰”效果，而且逻辑非常直观，不需要担心正负号问题。

    解决双 8000 问题：假设位置环都输出 8000。
    同步补偿算出：左+500，右-500。Raw Target：左 8500，右 7500。
    溢出量：左 +500，右 0。最大溢出：+500。修正后：左 8000，右 7000。
    
    结论：即使两边原本的基础输出都是 8000，这个算法依然能工作。
    这种写法将计算与执行分离，避免了重复计算速度环的 Bug，也让削峰逻辑变得健壮。
    */
    // ================= Step 1: 计算同步补偿量 =================
    float sync_comp[2] = {0, 0};
    
    // 仅在两个都进入位置模式时计算同步
    if (mode_deliver[0] == MODE_ANGLE && mode_deliver[1] == MODE_ANGLE) {
        // 计算位置差 (假设 R - L)
        float diff = DeliverMotor[1].getMotorTotalAngle() - DeliverMotor[0].getMotorTotalAngle();
        pid_deliver_sync.Target = 0;
        pid_deliver_sync.Current = diff;
        pid_deliver_sync.Adjust();

        // sync_comp[0] 加给左边, sync_comp[1] 加给右边
        sync_comp[0] =  pid_deliver_sync.Out;
        sync_comp[1] = -pid_deliver_sync.Out;
    } 
    else {
        pid_deliver_sync.clean_intergral();
    }

    // ================= Step 2: 计算位置环输出 & 预判速度目标 =================
    float raw_speed_target[2] = {0, 0}; // 暂存未削峰的目标速度
    bool is_saturated[2] = {false, false}; // 标记是否超限
    const float SPEED_LIMIT = DELIVER_PID_ANGLE_OUTMAX; // 速度限幅阈值

    for (int i = 0; i < 2; i++) {
        if(mode_deliver[i] == MODE_ANGLE){
            pid_deliver_pos[i].Target = std_lib::constrain(target_deliver_angle, POS_DELIVER_MIN, POS_DELIVER_MAX);
            pid_deliver_pos[i].Current = DeliverMotor[i].getMotorTotalAngle();
            pid_deliver_pos[i].Adjust();
            
            // 原始目标 = 位置环输出 + 同步补偿
            raw_speed_target[i] = pid_deliver_pos[i].Out + sync_comp[i];
            
            // 检查是否超过限幅
            if (fabs(raw_speed_target[i]) > SPEED_LIMIT) {
                is_saturated[i] = true;
            }
        }
    }

    // ================= Step 3: 统一削峰逻辑 (解决双边8000问题) =================
    if (mode_deliver[0] == MODE_ANGLE && mode_deliver[1] == MODE_ANGLE) {
        // 只要有一边饱和，就需要进行整体调整，维持相对差值
        if (is_saturated[0] || is_saturated[1]) {
            // 策略：找出溢出了多少，然后两边同时减去这个溢出量
            // 这样既能保证不超过限幅，又能维持 sync_comp 带来的速度差
            
            float overflow[2] = {0, 0};
            
            // 计算各自的溢出量 (保留符号)
            if (raw_speed_target[0] > SPEED_LIMIT)  overflow[0] = raw_speed_target[0] - SPEED_LIMIT;
            if (raw_speed_target[0] < -SPEED_LIMIT) overflow[0] = raw_speed_target[0] + SPEED_LIMIT;
            
            if (raw_speed_target[1] > SPEED_LIMIT)  overflow[1] = raw_speed_target[1] - SPEED_LIMIT;
            if (raw_speed_target[1] < -SPEED_LIMIT) overflow[1] = raw_speed_target[1] + SPEED_LIMIT;

            // 找出最严重的那个溢出量 (绝对值最大)
            // 比如左边溢出+500 (要减500)，右边没溢出，那就要两边都减500
            // 如果左边溢出+500，右边溢出+200，那就要两边都减500（也就是迁就最慢的那个限制）
            float max_overflow = 0;
            if (fabs(overflow[0]) > fabs(overflow[1])) {
                max_overflow = overflow[0];
            } else {
                max_overflow = overflow[1];
            }

            // 应用削峰：两边同时减去最大的溢出量
            raw_speed_target[0] -= max_overflow;
            raw_speed_target[1] -= max_overflow;
        }
    }

    // ================= Step 4: 执行速度环 =================
    for (int i = 0; i < 2; i++) {
        if(mode_deliver[i] == MODE_ANGLE){
            // 使用处理过后的目标值
            pid_deliver_spd[i].Target = raw_speed_target[i];
            pid_deliver_spd[i].Current = DeliverMotor[i].getMotorSpeed();
            pid_deliver_spd[i].Adjust();
        }
        else if(mode_deliver[i] == MODE_SPEED){
            pid_deliver_spd[i].Current = DeliverMotor[i].getMotorSpeed();
            pid_deliver_spd[i].Adjust();
        }
        else{
            // 失能
            DeliverMotor[i].setMotorCurrentOut(0);
            pid_deliver_pos[i].clean_intergral();
            pid_deliver_spd[i].clean_intergral();
            pid_deliver_pos[i].Out = 0;
            pid_deliver_spd[i].Target = 0;
            pid_deliver_spd[i].Out = 0;
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
        /*todo
        song
        每次撞爆限位开关都是速度环后面没有接校准，索性把校准逻辑放这里了
        然后也考虑上校准完成后误触发限位开关的情况，就算校准完成了再触发也重新校准一次。
        但是这样会导致校准完成后误触发限位开关会反复校准，这样稳定性极其依赖于限位开关的可靠性。
        所以还是先这样吧。
        */
        #if 0
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
        else{
            if (SW_IGNITER_OFF) {
                LOG_ERROR("Igniter Motor Homing Triggered (Switch Hit) when calibrated");
                pid_igniter_pos.clean_intergral();
                pid_igniter_spd.clean_intergral();
                IgniterMotor.baseAngle -= IgniterMotor.getMotorTotalAngle();
                mode_igniter = MODE_ANGLE;
                //pid_igniter_pos.Target=IGNITER_OFFSET_POS;
                target_igniter_angle = IGNITER_OFFSET_POS;
                is_igniter_homed = true;
            }
        }
        #endif
    }
    else{
        IgniterMotor.setMotorCurrentOut(0);
        pid_igniter_pos.Target=pid_igniter_pos.Current;
        pid_igniter_pos.Out=0;
        pid_igniter_spd.Target=0;
        pid_igniter_spd.Out=0;
        pid_igniter_pos.clean_intergral();
        pid_igniter_spd.clean_intergral();
    }
}

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
			DeliverMotor[0].baseAngle -= (DeliverMotor[0].getMotorTotalAngle()+Debugger.dual_loader_mechanical_error_correction);//双滑块机械装配误差修正
            // 2. 切换到位置模式
            mode_deliver[0] = MODE_ANGLE;
            // 3. 设定当前位置为回缓冲区
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

//自动终止发射状态机，需要无缝衔接发射流程
void Launcher_Driver::Abort_Firing_Sequence(){
    static uint32_t state_timer = 0;
    uint32_t current_time = xTaskGetTickCount();

    switch (fire_state)
    {
        //发射结束状态
        case FIRE_IDLE:
            Robot.Status.dart_count=0;
            Robot.Flag.Status.stop_continus_fire=true;
            //Robot.Status.current_state = SYS_AUTOFIRE_SUSPEND;
            Robot.Flag.Status.safely_abort_fire=false;
            break;
        
        //滑块下拉到扳机前的发射状态，此时中断自动发射只需要直接切换状态机到FIRE_IDLE即可
        case FIRE_PULL_DOWN_1:
        case FIRE_WAIT_BOTTOM_1:
        case FIRE_PULL_DOWN_2:
        case FIRE_WAIT_BOTTOM_2:
        case FIRE_RELOAD_LIFT_3:
        case FIRE_RELOAD_RELEASE_3:
        case FIRE_PULL_DOWN_3:
        case FIRE_WAIT_BOTTOM_3:
        case FIRE_RELOAD_LIFT_4:
        case FIRE_RELOAD_RELEASE_4:
        case FIRE_PULL_DOWN_4:
        case FIRE_WAIT_BOTTOM_4:
            loader_target_mode=LOAD_STOWED;//确保升降机在收起位置
            servo_transfomer_lock;// 锁止卡镖舵机
            servo_igniter_unlock; // 解锁扳机舵机，因为这里不希望上膛成功
            //但是扳机动作需要时间，这里加一个简单的非阻塞延时
            state_timer=current_time;
            LOG_ERROR("Firing Sequence Aborted during Pull Down Phase");
            fire_state=FIRE_ABORT_WAIT;
            break;
        
        case FIRE_ABORT_WAIT:
            if ((current_time - state_timer) > 300) {
                //滑块回缓冲区
                target_deliver_angle=(POS_BUFFER);
                if(Launcher.is_deliver_at_target(5))
                {
                    //滑块到位后，重置状态机
                    fire_state = FIRE_IDLE;
                    LOG_ERROR("Firing Sequence Aborted Completed");
                }
            }
            break;

        //滑块在校准状态中，完成校准状态，然后重置状态机
        case FIRE_CALIBRATION_1:
        case FIRE_CALIBRATION_2:
        case FIRE_CALIBRATION_3:
        case FIRE_CALIBRATION_4:
            //滑块继续完成校准并自动回到offset位置
            check_calibration_logic();
            if(is_calibrated()&&is_deliver_sync_ok(Debugger.deliver_sync_threshold)){
                //滑块到位后，重置状态机
                fire_state = FIRE_IDLE;
                LOG_INFO("Firing Sequence Aborted then Calibrate Completed");
            }
            break;
        
        //滑块已经下拉卡上扳机了，发射中断需要重新下拉，然后释放扳机，让滑块回缓冲区
        case FIRE_RETURN_UP_1:
        case FIRE_WAIT_UP_1:
        case FIRE_WAIT_AIM_1:
        case FIRE_RETURN_UP_2:
        case FIRE_WAIT_UP_2:
        case FIRE_WAIT_AIM_2:
        case FIRE_RETURN_UP_3:
        case FIRE_WAIT_UP_3:
        case FIRE_WAIT_AIM_3:
        case FIRE_RETURN_UP_4:
        case FIRE_WAIT_UP_4:
        case FIRE_WAIT_AIM_4:
            //滑块拉到底
            target_deliver_angle=(POS_BOTTOM);
            if(Launcher.is_deliver_at_target(5))
            {
                state_timer= current_time;
                servo_igniter_unlock; // 解锁扳机舵机，因为这里不希望上膛成功
                fire_state = FIRE_ABORT_WAIT;
                LOG_ERROR("Firing Sequence Aborted during locked Phase");
            }
            break;

        //已经进发射状态了，那没招了，打完得了。
        case FIRE_SHOOTING_1:
        case FIRE_SHOOTING_2:
        case FIRE_SHOOTING_3:
        case FIRE_SHOOTING_4:
            state_timer= current_time;
            //直接等发射完成
            servo_igniter_unlock; // 解锁扳机舵机，发射
            Robot.Status.dart_count++; // 计数+1
            fire_state=FIRE_ABORT_WAIT;
            LOG_ERROR("Firing Sequence Aborted during Shooting Phase");
            break;

        default:
            fire_state = FIRE_IDLE;
            LOG_ERROR("Firing Sequence Aborted due to Unknown State");
            break;
    }
}