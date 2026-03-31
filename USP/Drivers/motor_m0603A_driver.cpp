#include "motor_m0603A_driver.h"

BenMoMotor::BenMoMotor(uint8_t motor_id) : _id(motor_id) {}

/**
 * @brief CRC-8/MAXIM 校验实现 (多项式 x8 + x5 + x4 + 1)
 */
uint8_t BenMoMotor::calculateCRC8(const uint8_t* data, uint8_t len) {
    uint8_t crc = 0x00;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x01) {
                crc = (crc >> 1) ^ 0x8C; // 0x8C 是 0x31 的位反转
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

/**
 * @brief 通用包构建器 (10字节固定长度)
 */
std::vector<uint8_t> BenMoMotor::buildPacket(uint8_t cmd_type, uint16_t val, uint8_t data6, uint8_t data7) {
    std::vector<uint8_t> packet(10, 0);
    packet[0] = _id;
    packet[1] = cmd_type;
    packet[2] = (uint8_t)(val >> 8);   // 高8位
    packet[3] = (uint8_t)(val & 0xFF); // 低8位
    // DATA[4], DATA[5] 手册中发送基本都是默认为 0
    packet[4] = 0;
    packet[5] = 0;
    packet[6] = data6;
    packet[7] = data7;
    packet[9] = calculateCRC8(packet.data(), 9);
    return packet;
}

/**
 * @brief 切换运行模式 (对应手册 P10)
 * 模式值: 0x00 开环, 0x01 电流环, 0x02 速度环, 0x03 位置环
 */
std::vector<uint8_t> BenMoMotor::genModeCmd(MotorMode mode) {
    return buildPacket(0xA0, (uint16_t)mode << 8);
}

/**
 * @brief 使能/失能 (对应手册 P10 模式值 0x08/0x09)
 * 注意：使能/失能指令与切换模式指令共用寄存器 0xA0，DATA[2] 的值区分功能：
 * - 切换模式: DATA[2] = 0x00~0x03
 * - 使能/失能: DATA[2] = 0x08 (使能) 或 0x09 (失能)
 */
std::vector<uint8_t> BenMoMotor::genEnableCmd(bool enable) {
    return buildPacket(0xA0, (enable ? 0x08 : 0x09) << 8);
}

/**
 * @brief 速度控制 (对应手册 P8)
 * target_rpm: 目标转速(RPM)。例如输入30表示3rpm。
 * accel_time: 加速时间 (ms/1rpm), 0表示最快。默认1。
 */
std::vector<uint8_t> BenMoMotor::genSpeedCtrl(uint16_t target_rpm, uint8_t accel_time, bool brake) {
    // 转换规则：写入值 = 实际转速 * 10
    int16_t val = (int16_t)(target_rpm * 10.0f);
    return buildPacket(0x64, (uint16_t)val, accel_time, (brake ? 0xFF : 0x00));
}

/**
 * @brief 电流/力矩控制 (对应手册 P8)
 * current_raw: -32767 ~ 32767 (对应 -4A 到 4A)
 */
std::vector<uint8_t> BenMoMotor::genCurrentCtrl(int16_t current_raw) {
    return buildPacket(0x64, (uint16_t)current_raw);
}

/**
 * @brief 位置控制 (对应手册 P8)
 * position_value: 0~32767 对应 0~360°
 */
std::vector<uint8_t> BenMoMotor::genPositionCtrl(uint16_t position_value) {
    return buildPacket(0x64, position_value);
}

/**
 * @brief 查询额外反馈 (对应手册 P9)
 * 该指令请求电机返回里程、位置和当前模式等高级状态信息，反馈ID为 0x75/0x76。
 */
std::vector<uint8_t> BenMoMotor::genQueryExtraCmd() {
    return buildPacket(0x74, 0x0000);
}

/**
 * @brief 解析常规反馈：速度、电流、温度反馈 (对应手册 P8 反馈ID 0x65)
 * 反馈数据格式 (10字节):
    * Byte 0: 电机ID
    * Byte 1: 反馈ID (0x65)
    * Byte 2-3: 当前速度 (int16_t, 实际值 = speed / 10.0 RPM)
    * Byte 4-5: 当前电流 (int16_t, -32767~32767 对应 -4A~4A)
    * Byte 6: 保留 (手册中未定义，实际测试中通常为0)
    * Byte 7: 绕组温度 (uint8_t, 单位 ℃)
    * Byte 8: 故障码 (uint8_t, BIT0~BIT3: 过流、过速、过温、过压故障; BIT4~BIT6: 过温、断联、过欠压故障)
 */
bool BenMoMotor::parseDriveFeedback(const uint8_t* buf, MotorDriveStatus& out) {
    if (buf[0] != _id || buf[1] != 0x65) return false;
    if (calculateCRC8(buf, 9) != buf[9]) return false;

    out.speed = (int16_t)((buf[2] << 8) | buf[3]);   // 需要除以10得到真实RPM
    out.current = (int16_t)((buf[4] << 8) | buf[5]);
    out.temp = buf[7];
    out.fault_code = buf[8];
    return true;
}

/**
 * @brief 解析里程位置反馈 (对应手册 P9 反馈ID 0x75/0x76)
 * 反馈数据格式 (10字节):
    * Byte 0: 电机ID
    * Byte 1: 反馈ID (0x75 请求回复, 0x76 模式反馈)
    * Byte 2-5: 里程 (int32_t, 圈数，范围 -2,147,483,647 到 2,147,483,647)
    * Byte 6-7: 位置 (uint16_t, 0~32767 对应 0~360°)
    * Byte 8: 模式反馈时为当前模式值，其他情况保留
 */
bool BenMoMotor::parseExtraFeedback(const uint8_t* buf, MotorExtraStatus& out) {
    // 0x75 为请求回复，0x76 为模式反馈
    if (buf[0] != _id || (buf[1] != 0x75 && buf[1] != 0x76)) return false;
    if (calculateCRC8(buf, 9) != buf[9]) return false;

    if (buf[1] == 0x75) {
        // 里程 4 字节
        out.mileage = (int32_t)((buf[2] << 24) | (buf[3] << 16) | (buf[4] << 8) | buf[5]);
        // 位置 2 字节
        out.position = (uint16_t)((buf[6] << 8) | buf[7]);
    } else {
        // 0x76 模式反馈包
        out.mode = buf[2];
    }
    return true;
}