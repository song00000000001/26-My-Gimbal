#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief PID工作模式
 */
typedef enum
{
    MY_PID_MODE_POSITION = 0,   ///< 位置式PID
    MY_PID_MODE_INCREMENTAL,    ///< 增量式PID
    MY_PID_MODE_GIMBAL_INC_POS  ///< 云台专用：输出位置增量，再累加为目标位置
} MyPidMode;

/**
 * @brief PID限幅配置
 */
typedef struct
{
    float out_min;          ///< 最终输出下限
    float out_max;          ///< 最终输出上限
    float integ_min;        ///< 积分项下限
    float integ_max;        ///< 积分项上限
    float integ_split_threshold; ///< 积分分离阈值
    float delta_out_min;    ///< 增量输出下限（增量式/云台模式用）
    float delta_out_max;    ///< 增量输出上限（增量式/云台模式用）
} MyPidLimit;

/**
 * @brief PID参数
 */
typedef struct
{
    float kp;
    float ki;
    float kd;
} MyPidParam;

/**
 * @brief PID运行时数据
 */
typedef struct
{
    float ref;              ///< 参考值
    float fdb;              ///< 反馈值
    float err;              ///< 当前误差 e[k]
    float err_last;         ///< 上次误差 e[k-1]
    float err_last2;        ///< 上上次误差 e[k-2]

    float pout;             ///< 比例项
    float iout;             ///< 积分项
    float dout;             ///< 微分项

    float out;              ///< 当前输出
    float out_last;         ///< 上次输出

    float delta_out;        ///< 当前增量输出
    float delta_out_last;   ///< 上次增量输出

    float target_accum;     ///< 云台模式：累计后的位置目标
} MyPidData;

/**
 * @brief PID控制器对象
 */
typedef struct
{
    MyPidMode mode;
    MyPidParam param;
    MyPidLimit limit;
    MyPidData data;

    float dt;               ///< 控制周期，单位 s
    bool integ_enable;      ///< 是否启用积分
    bool integ_split_enable;///< 是否启用积分分离
    bool d_split_enable;    ///< 是否启用微分分离（微分项只对测量值求导，避免参考值突变引起微分峰值）
} MyPid;

/**
 * @brief 初始化PID对象
 */
void MyPid_Init(MyPid *pid,
                MyPidMode mode,
                float kp,
                float ki,
                float kd,
                float dt);

/**
 * @brief 设置PID参数
 */
void MyPid_SetParam(MyPid *pid, float kp, float ki, float kd);

/**
 * @brief 设置PID输出限幅
 */
void MyPid_SetLimit(MyPid *pid,
                    float out_min,
                    float out_max,
                    float integ_min,
                    float integ_max,
                    float delta_out_min,
                    float delta_out_max);

/**
 * @brief 复位PID内部状态
 */
void MyPid_Reset(MyPid *pid);

/**
 * @brief 计算PID输出
 * @param pid PID对象
 * @param ref 参考值
 * @param fdb 反馈值
 * @return float 控制输出
 */
float MyPid_Calc(MyPid *pid, float ref, float fdb);

/**
 * @brief 云台专用：角度误差 + 陀螺仪阻尼，输出位置目标
 * @param pid 必须使用 MY_PID_MODE_GIMBAL_INC_POS
 * @param angle_ref 目标角
 * @param angle_fdb 当前角
 * @param gyro_dps 角速度反馈（deg/s）
 * @return float 累加后的目标位置
 */
float MyPid_CalcGimbal(MyPid *pid, float angle_ref, float angle_fdb, float gyro_dps);

/**
 * @brief 直接设置云台模式下的累计目标位置
 */
void MyPid_SetAccumTarget(MyPid *pid, float target);

/**
 * @brief 对角度误差做环绕处理，例如 -180~180
 */
float MyPid_AngleWrapDeg(float err_deg);

#ifdef __cplusplus
}
#endif
