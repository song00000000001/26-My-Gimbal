#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    MOTOR_SPEED_STATE_STOP = 0,
    MOTOR_SPEED_STATE_STARTING,
    MOTOR_SPEED_STATE_RUNNING
} MotorSpeedState;

typedef struct
{
    float stop_rpm_th;
    float stop_ref_th;
    float starting_rpm_th;
    float running_rpm_th;
    uint32_t running_hold_ms;
    uint32_t startup_timeout_ms;

    float dir_ref_th;
    float dir_iq_th;

    float i_start_min;
    float i_start_boost;
    float i_fric_run;
    float iq_cmd_limit;
} MotorSpeedCtrlParam;

typedef struct
{
    MotorSpeedState state;
    uint32_t starting_elapsed_ms;
    uint32_t running_confirm_ms;
} MotorSpeedCtrl;

typedef struct
{
    float iq_ref_base;
    float iq_cmd;
    float dir;
    bool startup_timeout;
    MotorSpeedState state;
} MotorSpeedCtrlOutput;

static inline float motor_speed_ctrl_absf(float x)
{
    return (x >= 0.0f) ? x : -x;
}

static inline float motor_speed_ctrl_clamp(float x, float min_val, float max_val)
{
    if (x > max_val) return max_val;
    if (x < min_val) return min_val;
    return x;
}

static inline float motor_speed_ctrl_sign_with_th(float x, float th)
{
    if (x > th) return 1.0f;
    if (x < -th) return -1.0f;
    return 0.0f;
}

static inline float motor_speed_ctrl_get_dir(float speed_ref_rpm,
                                             float iq_ref_base,
                                             const MotorSpeedCtrlParam *param)
{
    float dir = motor_speed_ctrl_sign_with_th(speed_ref_rpm, param->dir_ref_th);
    if (dir == 0.0f)
    {
        dir = motor_speed_ctrl_sign_with_th(iq_ref_base, param->dir_iq_th);
    }
    return dir;
}

static inline void motor_speed_ctrl_reset(MotorSpeedCtrl *ctrl)
{
    if (ctrl == 0) return;
    ctrl->state = MOTOR_SPEED_STATE_STOP;
    ctrl->starting_elapsed_ms = 0U;
    ctrl->running_confirm_ms = 0U;
}

static inline void motor_speed_ctrl_update(MotorSpeedCtrl *ctrl,
                                           const MotorSpeedCtrlParam *param,
                                           float speed_ref_rpm,
                                           float speed_meas_rpm,
                                           float iq_ref_base,
                                           uint32_t dt_ms,
                                           MotorSpeedCtrlOutput *out)
{
    if (ctrl == 0 || param == 0 || out == 0) return;

    const float abs_rpm = motor_speed_ctrl_absf(speed_meas_rpm);
    const bool target_near_zero = (motor_speed_ctrl_absf(speed_ref_rpm) < param->stop_ref_th);
    const float dir = motor_speed_ctrl_get_dir(speed_ref_rpm, iq_ref_base, param);
    const bool has_target = (dir != 0.0f) && !target_near_zero;

    out->iq_ref_base = iq_ref_base;
    out->iq_cmd = iq_ref_base;
    out->dir = dir;
    out->startup_timeout = false;

    switch (ctrl->state)
    {
    case MOTOR_SPEED_STATE_STOP:
        ctrl->starting_elapsed_ms = 0U;
        ctrl->running_confirm_ms = 0U;
        if (has_target && abs_rpm < param->starting_rpm_th)
        {
            ctrl->state = MOTOR_SPEED_STATE_STARTING;
        }
        else if (has_target && abs_rpm > param->running_rpm_th)
        {
            ctrl->state = MOTOR_SPEED_STATE_RUNNING;
        }
        break;

    case MOTOR_SPEED_STATE_STARTING:
        if (!has_target && abs_rpm < param->stop_rpm_th)
        {
            ctrl->state = MOTOR_SPEED_STATE_STOP;
            ctrl->starting_elapsed_ms = 0U;
            ctrl->running_confirm_ms = 0U;
            break;
        }

        ctrl->starting_elapsed_ms += dt_ms;

        if (abs_rpm > param->running_rpm_th)
        {
            ctrl->running_confirm_ms += dt_ms;
            if (ctrl->running_confirm_ms >= param->running_hold_ms)
            {
                ctrl->state = MOTOR_SPEED_STATE_RUNNING;
                ctrl->starting_elapsed_ms = 0U;
                ctrl->running_confirm_ms = 0U;
            }
        }
        else
        {
            ctrl->running_confirm_ms = 0U;
        }

        if (ctrl->state == MOTOR_SPEED_STATE_STARTING &&
            ctrl->starting_elapsed_ms > param->startup_timeout_ms &&
            abs_rpm < param->starting_rpm_th)
        {
            out->startup_timeout = true;
            out->iq_cmd = 0.0f;
            ctrl->state = MOTOR_SPEED_STATE_STOP;
            ctrl->starting_elapsed_ms = 0U;
            ctrl->running_confirm_ms = 0U;
            out->state = ctrl->state;
            return;
        }
        break;

    case MOTOR_SPEED_STATE_RUNNING:
        ctrl->starting_elapsed_ms = 0U;
        ctrl->running_confirm_ms = 0U;
        if (!has_target && abs_rpm < param->stop_rpm_th)
        {
            ctrl->state = MOTOR_SPEED_STATE_STOP;
        }
        else if (has_target && abs_rpm < param->starting_rpm_th)
        {
            ctrl->state = MOTOR_SPEED_STATE_STARTING;
        }
        break;

    default:
        motor_speed_ctrl_reset(ctrl);
        break;
    }

    if (ctrl->state == MOTOR_SPEED_STATE_STARTING && dir != 0.0f)
    {
        out->iq_cmd = iq_ref_base + dir * (param->i_start_boost + param->i_fric_run);
        if (motor_speed_ctrl_absf(out->iq_cmd) < param->i_start_min)
        {
            out->iq_cmd = dir * param->i_start_min;
        }
    }
    else if (ctrl->state == MOTOR_SPEED_STATE_RUNNING && dir != 0.0f)
    {
        out->iq_cmd = iq_ref_base + dir * param->i_fric_run;
    }

    out->iq_cmd = motor_speed_ctrl_clamp(out->iq_cmd, -param->iq_cmd_limit, param->iq_cmd_limit);
    out->state = ctrl->state;
}

#ifdef __cplusplus
}
#endif
