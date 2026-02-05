#include "remote_ctrl_driver.h"
#include "global_data.h"
#include "robot_config.h"


/**
 * @brief 摇杆步进控制函数（带蜂鸣器反馈）
 * @param input     当前摇杆的归一化输入 (-1.0 到 1.0)
 * @param state     指向该摇杆对应的状态结构体
 * @param target    指向要修改的变量
 * @param min_val   变量最小值
 * @param max_val   变量最大值
 */
int Step_Control_With_Feedback(float input, JoystickTrigger_t *state, int *target, int min_val, int max_val) {
    const float threshold = 0.85f; // 触发阈值
    int delta = 0;

    // --- 1. 边缘检测逻辑 ---
    
    // 正向触发检测 (从 0 到 1 的瞬间)
    bool current_pushed_pos = (input > threshold);
    if (current_pushed_pos && !state->last_pushed_pos) {
        delta = 1;
    }
    state->last_pushed_pos = current_pushed_pos;

    // 负向触发检测 (从 0 到 -1 的瞬间)
    bool current_pushed_neg = (input < -threshold);
    if (current_pushed_neg && !state->last_pushed_neg) {
        delta = -1;
    }
    state->last_pushed_neg = current_pushed_neg;

    // --- 2. 变量修改与反馈 ---
    if (delta != 0) {
        int next_val = *target + delta;

        // 边界检查
        if (next_val >= min_val && next_val <= max_val) {
            *target = next_val;
          }
    }

    return delta;
}

//遥控器快照数据拷贝函数
void Remote_Ctrl_Snapshot_Copy(FS_I6X_Snapshot_t *dest, FS_I6X_Classdef *src) {
    // 尝试拿锁，参数为 0 表示：拿不到立刻返回，不等待，不阻塞
    //if (xSemaphoreTake(FS_I6X_mutex, 0) == pdTRUE)
    //{
    // 1. 拿到了锁：更新快照
    dest->Status = src->GetStatus();
    dest->LX_Norm = src->Get_LX_Norm();
    dest->LY_Norm = src->Get_LY_Norm();
    dest->RX_Norm = src->Get_RX_Norm();
    dest->RY_Norm = src->Get_RY_Norm();
    dest->S1 = src->Get_SWA();
    dest->S2 = src->Get_SWB();
    dest->S3 = src->Get_SWC();
    dest->S4 = src->Get_SWD();
    dest->VRA_Norm = src->Get_VRA_Norm();
    dest->VRB_Norm = src->Get_VRB_Norm();
    // 2. 释放锁
    //    xSemaphoreGive(FS_I6X_mutex);
    //}
}

//遥控器数据快照
FS_I6X_Snapshot_t FS_I6X_Snap; 

JoystickTrigger_t Joystick_LX_Trigger={false,false};
JoystickTrigger_t Joystick_LY_Trigger={false,false};
