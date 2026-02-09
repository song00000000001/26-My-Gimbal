#pragma once

#ifdef __cplusplus

#include "SRML.h"


/**
 * @brief 遥杆步进触发器类
 */
class JoystickStepper {
public:
    float threshold = 0.8f;   // 触发阈值
    bool last_pushed_pos = false; // 上一次是否正向推到底
    bool last_pushed_neg = false; // 上一次是否负向推到底

    /**
     * @brief 更新并检测触发情况
     * @param input 当前遥杆归一化输入 (-1.0 到 1.0)
     * @return int 触发增量：1(正向触发), -1(负向触发), 0(无触发)
     */
    int update(float input) {
        int delta = 0;
        
        // 正向检测 (例如 LX > 0.8)
        bool current_pushed_pos = (input > threshold);
        if (current_pushed_pos && !last_pushed_pos) {
            delta = 1;
        }
        last_pushed_pos = current_pushed_pos;

        // 负向检测 (例如 LX < -0.8)
        bool current_pushed_neg = (input < -threshold);
        if (current_pushed_neg && !last_pushed_neg) {
            delta = -1;
        }
        last_pushed_neg = current_pushed_neg;

        return delta;
    }
};

//FS_I6X快照结构体
struct FS_I6X_Snapshot_t {
    LinkageStatus_Typedef Status;
    SW_Status_Typedef S1;
    SW_Status_Typedef S2;
    SW_Status_Typedef S3;
    SW_Status_Typedef S4;
    float VRA_Norm;
    float VRB_Norm;
    float RX_Norm;
    float RY_Norm;
    float LX_Norm;
    float LY_Norm;
};
extern FS_I6X_Snapshot_t FS_I6X_Snap; 

/**
 * @brief 摇杆触发器状态记录
 */
typedef struct {
    bool last_pushed_pos; // 上一次是否推向正向(>0.8)
    bool last_pushed_neg; // 上一次是否推向负向(<-0.8)
} JoystickTrigger_t;
extern JoystickTrigger_t Joystick_LX_Trigger;
extern JoystickTrigger_t Joystick_LY_Trigger;


void Remote_Ctrl_Snapshot_Copy(FS_I6X_Snapshot_t *dest, FS_I6X_Classdef *src);
// 摇杆带触发器的阶梯控制函数
int Step_Control_With_Feedback(float input, JoystickTrigger_t *state, int *target, int min_val, int max_val);

#endif