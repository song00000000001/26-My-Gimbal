#ifndef DM_MOTOR_H
#define DM_MOTOR_H

#ifdef __cplusplus
#include "srml_std_lib.h"
#include "Drivers/Components/drv_can.h"
#include "FreeRTOS.h"
#include "queue.h"

enum damiao_state
{
  HighVoltage = 8,
  LowVoltage = 9,
  Over_Current = 'A',
  HighMOSTemperature = 'B',
  HighRototTemperature = 'C',
  LostConnect = 'D',
  OverLoad = 'E'
};

class Motor_DM_classdef
{
private:
    /* 数据池 */
    int32_t state = 0; /* 电机状态 */

    const int32_t encoder_max = 65536; /* 码盘值最大值 */
    bool encoder_is_init = false;
    int32_t round_cnt = 0;
    uint16_t encoder, encoder_offset = 0, last_encoder = 0;

    struct RecData_Structdef_
    {
        float angle = 0;   // 多圈角度(°)
        float position;    // 单圈位置(rad)
        float velocity;    // 当前速度(rad/s)
        float torque = 0;  // 当前力矩(N*m)
        float T_mos = 0;   // 表示驱动上 MOS 的平均温度
        float T_rotor = 0; // 表示电机内部线圈的平均温度
    } Rec_Data;

    /* can发送数据体 */
    CAN_COB TxPack = {Can_STDID, 0, 8, {}};
    QueueHandle_t Tx_Handle;

public:
    const uint8_t ID;
    Motor_DM_classdef(uint8_t _id) : ID(_id) { TxPack.ID = ID; }
    inline void bindCanQueueHandle(QueueHandle_t _sendQueue) { Tx_Handle = _sendQueue; }

    void startMotor(); /* 发送使能帧给电机 */
    void stopMotor();  /* 发送失能帧给电机 */
    void ClearError(); /* 清除错误标志位 */

    /* 设置电机力矩，内置can发包 */
    void control(float position, float velocity, float kp, float kd, float torque);
    inline void setTorque(float torque) { control(0, 0, 0, 0, torque); }                                          // 力矩控制
    inline void setPosition(float _position, float _kp, float _kd) { control(_position, 0, _kp, _kd, 0); }        // 内置位置控制
    inline void setSpeed(float _velocity, float _kd, float torque = 0) { control(0, _velocity, 0, _kd, torque); } // 内置速度控制

    bool update(uint32_t _unuse_id, uint8_t data[8]);
    inline const RecData_Structdef_ &getRecData() { return Rec_Data; }
    /* 设置编码器offset */
    void setEncoderOffset(uint16_t offset);
    float Out = 0; // 用于存储计算出的控制量（电流/扭矩）

private:
    void update_angle(uint8_t can_rx_data[]);
    /* 需要的常量定义 */
    const float P_MIN = -12.5;
    const float P_MAX = 12.5f;
    const float V_MIN = -60.0;
    const float V_MAX = 60.0f;
    const float T_MIN = -10.0;
    const float T_MAX = 10.0f;
    const float KP_MIN = 0.0f;
    const float KP_MAX = 500.;
    const float KD_MIN = 0.0f;
    const float KD_MAX = 5.0f;
};
#endif /* __cplusplus */

#endif /* DM_MOTOR_H */
