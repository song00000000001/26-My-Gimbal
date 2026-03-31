#pragma once

#include <stdint.h>
#include <vector>

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
    std::vector<uint8_t> genModeCmd(MotorMode mode);

    // 生成使能/失能指令 (0xA0 0x08/0x09)
    std::vector<uint8_t> genEnableCmd(bool enable);

    // 生成速度控制指令 (0x64)
    // target_rpm: 目标转速(0.1RPM)，例如输入30表示3rpm。
    // accel_time: 加速时间 (ms/1rpm), 0表示最快。默认1。
    // brake: 是否刹车 (仅速度环有效)
    std::vector<uint8_t> genSpeedCtrl(uint16_t target_rpm, uint8_t accel_time = 0, bool brake = false);

    // 生成电流/力矩控制指令 (0x64)
    // current_raw: -32767 ~ 32767 (对应 -4A 到 4A)
    std::vector<uint8_t> genCurrentCtrl(int16_t current_raw);

    // 生成位置控制指令 (0x64)
    std::vector<uint8_t> genPositionCtrl(uint16_t position_value);

    // 生成请求其他反馈指令 (0x74) - 获取里程和精确位置
    std::vector<uint8_t> genQueryExtraCmd();

    // ==================== [数据解析逻辑] ====================

    // 解析 ID 0x65 的反馈数据 (速度、电流、温度)
    bool parseDriveFeedback(const uint8_t* buf, MotorDriveStatus& out);

    // 解析 ID 0x75/0x76 的反馈数据 (里程、位置、模式)
    bool parseExtraFeedback(const uint8_t* buf, MotorExtraStatus& out);

    // CRC8 校验计算 (CRC-8/MAXIM)
    static uint8_t calculateCRC8(const uint8_t* data, uint8_t len);

    private:
    uint8_t _id;
    // 基础封包函数
    std::vector<uint8_t> buildPacket(uint8_t reg, uint16_t val, uint8_t d6 = 0, uint8_t d7 = 0);
};

