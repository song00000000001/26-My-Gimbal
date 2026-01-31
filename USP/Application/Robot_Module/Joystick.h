#pragma once

#ifdef __cplusplus

#include "SRML.h"
#include "robot_types.h"

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

#endif