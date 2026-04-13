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
    float delta_out_min;    ///< 增量输出下限（增量式用）
    float delta_out_max;    ///< 增量输出上限（增量式用）
} MyPidLimit;

/**
 * @brief PID参数
 */
typedef struct
{
    float kp;
    float ki;
    float kd;
    float kff; // 前馈增益
} MyPidParam;

/**
 * @brief PID运行时数据
 */
typedef struct
{
    float ref;              ///< 参考值
    float fdb;              ///< 反馈值
    float fdb_last;         ///< 上次反馈值
    float fdb_last2;        ///< 上上次反馈值
    float err;              ///< 当前误差 e[k]
    float err_last;         ///< 上次误差 e[k-1]
    float err_last2;        ///< 上上次误差 e[k-2]

    float pout;             ///< 比例项
    float iout;             ///< 积分项
    float dout;             ///< 微分项
    float ffout;            ///< 前馈项

    float out;              ///< 当前输出
    float out_last;         ///< 上次输出

    float delta_out;        ///< 当前增量输出
    float delta_out_last;   ///< 上次增量输出

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
    float param_scale;      ///< 参数缩放，便于调参时使用较大的整数参数，内部自动缩放到实际值
    bool integ_enable;      ///< 是否启用积分
    bool d_split_enable;    ///< 是否启用微分分离（微分项只对测量值求导，避免参考值突变引起微分峰值）
} MyPid;

/**
 * @brief 初始化PID对象
 */
void MyPid_Init(MyPid *pid,
                MyPidMode mode=MY_PID_MODE_POSITION,
                float param_scale=1.0f,
                float dt=0.007f);

/**
 * @brief 设置PID参数
 * @param pid PID对象指针
 * @param kp 比例增益
 * @param ki 积分增益   
 * @param kd 微分增益
 * @param kff 前馈增益
 * @details 内部会将参数乘以pid->param_scale进行缩放，方便调参时使用较大的整数参数，例如1000，实际生效的增益会是0.001倍
 */
void MyPid_SetParam(MyPid *pid, float kp, float ki, float kd, float kff);

/**
 * @brief 设置PID参数（结构体版本）
 * @param pid PID对象指针
 * @param param PID参数结构体指针
 */
void MyPid_SetParam_Struct(MyPid *pid, MyPidParam *param);

/**
 * @brief 设置PID输出限幅
 */
void MyPid_SetLimit(MyPid *pid,
                    float out_min,
                    float out_max,
                    float integ_min=0.0f,
                    float integ_max=0.0f,
                    float delta_out_min=0.0f,
                    float delta_out_max=0.0f);

/**
 * @brief 设置积分分离阈值（绝对误差超过阈值时暂停积分）
 */
void MyPid_SetIntegSplitThreshold(MyPid *pid, float threshold);

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
 * @brief 对角度误差做环绕处理，例如 -180~180
 */
float MyPid_AngleWrapDeg(float err_deg);

#ifdef __cplusplus
}
#endif
