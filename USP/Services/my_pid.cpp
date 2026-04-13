#include "my_pid.h"

#include <math.h>

static float my_clamp(float x, float min_val, float max_val)
{
    if (x > max_val) return max_val;
    if (x < min_val) return min_val;
    return x;
}

float MyPid_AngleWrapDeg(float err_deg)
{
    while (err_deg > 180.0f) err_deg -= 360.0f;
    while (err_deg < -180.0f) err_deg += 360.0f;
    return err_deg;
}

void MyPid_Init(MyPid *pid,
                MyPidMode mode,
                float kp,
                float ki,
                float kd,
                float dt)
{
    if (pid == 0) return;

    pid->mode = mode;
    pid->param.kp = kp;
    pid->param.ki = ki;
    pid->param.kd = kd;
    pid->dt = dt;

    pid->integ_enable = true;
    pid->d_split_enable = true;
    
    pid->limit.out_min = 0;
    pid->limit.out_max =  0;
    pid->limit.integ_min = 0;
    pid->limit.integ_max =  0;
    pid->limit.integ_split_threshold = 0;
    pid->limit.delta_out_min = 0;
    pid->limit.delta_out_max =  0;

    MyPid_Reset(pid);
}

void MyPid_SetParam(MyPid *pid, float kp, float ki, float kd)
{
    if (pid == 0) return;
    pid->param.kp = kp;
    pid->param.ki = ki;
    pid->param.kd = kd;
}

void MyPid_SetLimit(MyPid *pid,
                    float out_min,
                    float out_max,
                    float integ_min,
                    float integ_max,
                    float delta_out_min,
                    float delta_out_max)
{
    if (pid == 0) return;

    pid->limit.out_min = out_min;
    pid->limit.out_max = out_max;
    pid->limit.integ_min = integ_min;
    pid->limit.integ_max = integ_max;
    pid->limit.delta_out_min = delta_out_min;
    pid->limit.delta_out_max = delta_out_max;
}

/**
 * @brief 设置积分分离阈值（绝对误差超过阈值时暂停并清空积分）
 * @param pid PID对象指针
 * @param threshold 阈值，单位与误差相同。如果设置为0则禁用。负数自动转换为正数。
 */
void MyPid_SetIntegSplitThreshold(MyPid *pid, float threshold)
{
    if (pid == 0) return;
    pid->limit.integ_split_threshold = (threshold >= 0.0f) ? threshold : -threshold;
}

void MyPid_Reset(MyPid *pid)
{
    if (pid == 0) return;

    pid->data.ref = 0.0f;
    pid->data.fdb = 0.0f;
    pid->data.fdb_last = 0.0f;
    pid->data.fdb_last2 = 0.0f;
    pid->data.err = 0.0f;
    pid->data.err_last = 0.0f;
    pid->data.err_last2 = 0.0f;

    pid->data.pout = 0.0f;
    pid->data.iout = 0.0f;
    pid->data.dout = 0.0f;

    pid->data.out = 0.0f;
    pid->data.out_last = 0.0f;

    pid->data.delta_out = 0.0f;
    pid->data.delta_out_last = 0.0f;
}

float MyPid_Calc(MyPid *pid, float ref, float fdb)
{
    if (pid == 0) return 0.0f;

    pid->data.ref = ref;

    pid->data.fdb_last2 = pid->data.fdb_last;
    pid->data.fdb_last  = pid->data.fdb;
    //pid->data.fdb = MyPid_AngleWrapDeg(fdb);
    /**
     * @todo 想清楚环绕处理后再写
     */
    pid->data.fdb = fdb;

    pid->data.err_last2 = pid->data.err_last;
    pid->data.err_last  = pid->data.err;
    pid->data.err       = ref - fdb;

    switch (pid->mode)
    {
    case MY_PID_MODE_POSITION:
    {
        bool integ_allow = pid->integ_enable;
        if (integ_allow)
        {
            float th = pid->limit.integ_split_threshold;
            if (th > 0.0f && fabsf(pid->data.err) > th)
            {
                integ_allow = false;
                pid->data.iout = 0.0f;//误差过大时清除积分，避免积分影响到后续的控制
            }
        }

        pid->data.pout = pid->param.kp * pid->data.err;

        if (integ_allow)
        {
            pid->data.iout += pid->param.ki * pid->data.err * pid->dt;
            pid->data.iout = my_clamp(pid->data.iout,
                                      pid->limit.integ_min,
                                      pid->limit.integ_max);
        }
        else if (!pid->integ_enable)
        {
            pid->data.iout = 0.0f;
        }

        if (pid->d_split_enable)
        {
            pid->data.dout = -pid->param.kd *
                             (pid->data.fdb - pid->data.fdb_last) / pid->dt;
        }
        else
        {
            pid->data.dout = pid->param.kd *
                             (pid->data.err - pid->data.err_last) / pid->dt;
        }

        pid->data.out = pid->data.pout + pid->data.iout + pid->data.dout;
        pid->data.out = my_clamp(pid->data.out,
                                 pid->limit.out_min,
                                 pid->limit.out_max);
    }
    break;

    case MY_PID_MODE_INCREMENTAL:
    {
        bool integ_allow = pid->integ_enable;
        if (integ_allow)
        {
            float th = pid->limit.integ_split_threshold;
            if (th > 0.0f && fabsf(pid->data.err) > th)//如果设置了非零的积分分离阈值，则启用积分分离
            {
                integ_allow = false;
            }
        }

        float de   = pid->data.err - pid->data.err_last;
        float dde  = pid->data.err - 2.0f * pid->data.err_last + pid->data.err_last2;
        float dd_fdb = pid->data.fdb - 2.0f * pid->data.fdb_last + pid->data.fdb_last2;

        pid->data.pout = pid->param.kp * de;

        if (integ_allow)
        {
            pid->data.iout = pid->param.ki * pid->data.err * pid->dt;
        }
        else
        {
            pid->data.iout = 0.0f;
        }

        if (pid->d_split_enable)
        {
            pid->data.dout = -pid->param.kd * dd_fdb / pid->dt;
        }
        else
        {
            pid->data.dout = pid->param.kd * dde / pid->dt;
        }

        pid->data.delta_out = pid->data.pout + pid->data.iout + pid->data.dout;
        pid->data.delta_out = my_clamp(pid->data.delta_out,
                                       pid->limit.delta_out_min,
                                       pid->limit.delta_out_max);

        pid->data.out = pid->data.out_last + pid->data.delta_out;
        pid->data.out = my_clamp(pid->data.out,
                                 pid->limit.out_min,
                                 pid->limit.out_max);
    }
    break;

    default:
        pid->data.out = 0.0f;
        break;
    }

    pid->data.out_last = pid->data.out;
    pid->data.delta_out_last = pid->data.delta_out;

    return pid->data.out;
}
