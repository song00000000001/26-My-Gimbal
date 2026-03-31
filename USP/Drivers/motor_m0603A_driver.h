#pragma once

#include <stdint.h>
#include <string.h> // 使用 memset

#define MOTOR_PACKET_SIZE 10 // 电机通信包固定为10字节
/**
 * @brief 电机运行模式枚举 (对应手册 P10/P11)
 */
enum class MotorMode : uint8_t {
    OPEN_LOOP     = 0x00, // 开环模式
    CURRENT_LOOP  = 0x01, // 电流环模式 (力矩控制)
    SPEED_LOOP    = 0x02, // 速度环模式
    POSITION_LOOP = 0x03  // 位置环模式
};

/**
 * @brief 电机基础反馈数据结构 (对应手册 P8)
 * @details
 * fault_code 的位定义 (BIT0~BIT6):
 * BIT0: 霍尔故障
 * BIT1: 过流故障
 * BIT3: 堵转故障
 * BIT4: 过温故障
 * BIT5: 断联故障
 * BIT6: 过欠压故障
 */
struct MotorDriveStatus {
    int16_t speed;      // 当前速度: 实际值 = speed / 10.0 (RPM)
    int16_t current;    // 当前电流: -32767~32767 对应 -4A~4A
    uint8_t temp;       // 绕组温度: 单位 ℃
    uint8_t fault_code; // 故障码: 详见故障位定义
};


/**
 * @brief 电机高级反馈数据结构 (对应手册 P9)
 */
struct MotorExtraStatus {
    int32_t mileage;    // 里程圈数: 范围 -2,147,483,647 到 2,147,483,647
    uint16_t position;  // 位置值: 0~32767 对应 0~360°
    uint8_t mode;       // 当前反馈的运行模式
};

class BenMoMotor {
public:
    /**
     * @param motor_id 电机ID (通常为 1 或 2)
    */
    BenMoMotor(uint8_t motor_id);

    // ==================== [发送指令生成] ====================

    // 生成切换模式指令 (0xA0)
    void genModeCmd(uint8_t* out_packet, uint8_t mode_val);

    // 生成使能/失能指令 (0xA0 0x08/0x09)
    void genEnableCmd(uint8_t* out_packet, bool enable);

    // 生成速度控制指令 (0x64)
    // target_rpm: 目标转速(0.1RPM)，例如输入30表示3rpm。
    // accel_time: 加速时间 (ms/1rpm), 0表示最快。默认1。
    // brake: 是否刹车 (仅速度环有效)
    void genSpeedCtrl(uint8_t* out_packet, float target_rpm, uint8_t accel_time = 0, bool brake = false);

    // 生成电流/力矩控制指令 (0x64)
    // current_raw: -32767 ~ 32767 (对应 -4A 到 4A)
    void genCurrentCtrl(uint8_t* out_packet, int16_t current_raw);

    // 生成位置控制指令 (0x64)
    void genPositionCtrl(uint8_t* out_packet, uint16_t position_value);

    // 生成请求其他反馈指令 (0x74) - 获取里程和精确位置
    void genQueryExtraCmd(uint8_t* out_packet);

    // ==================== [数据解析逻辑] ====================

    // 解析 ID 0x65 的反馈数据 (速度、电流、温度)
    bool parseDriveFeedback(const uint8_t* buf, MotorDriveStatus& out);

    // 解析 ID 0x75/0x76 的反馈数据 (里程、位置、模式)
    bool parseExtraFeedback(const uint8_t* buf, MotorExtraStatus& out);

    // CRC8 校验计算 (CRC-8/MAXIM)
    static uint8_t calculateCRC8(const uint8_t* data, uint8_t len);

    //获取反馈数据
    int16_t getSpeed() const { return __drive_status.speed; }
    int16_t getCurrent() const { return __drive_status.current; }
    uint8_t getTemp() const { return __drive_status.temp; }
    uint8_t getFaultCode() const { return __drive_status.fault_code; }
    int32_t getMileage() const { return __extra_status.mileage; }
    uint16_t getPosition() const { return __extra_status.position; }
    uint8_t getMode() const { return __extra_status.mode; }
    private:
    uint8_t _id;
    MotorDriveStatus __drive_status; // 上次的常规反馈状态
    MotorExtraStatus __extra_status; // 上次的额外反馈状态
    // 内部通用的填包逻辑
    void buildBasicPacket(uint8_t* buf, uint8_t reg, uint16_t val, uint8_t d6 = 0, uint8_t d7 = 0);
};

