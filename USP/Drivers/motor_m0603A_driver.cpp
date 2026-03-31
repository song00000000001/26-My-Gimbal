#include "motor_m0603A_driver.h"

// 静态成员变量定义和初始化
bool (*BenMoMotor::_sendFunction)(uint8_t, const uint8_t*, uint8_t) = nullptr;

BenMoMotor::BenMoMotor(uint8_t motor_id) : _id(motor_id) {}

/**
 * @brief 注册串口发送函数
 * @param sendFunc 用户提供的发送函数指针，函数签名为 bool sendFunc(uint8_t port_id, const uint8_t* data, uint8_t len)，用于发送生成的指令包
 * 该函数需要在使用驱动前调用一次，注册后驱动内部会使用该函数发送指令包，用户无需关心通信细节，只需提供一个符合签名的发送函数即可。
 */
void BenMoMotor::registerSendFunction(bool (*sendFunc)(uint8_t port_id,const uint8_t* data, uint8_t len)) {
    // 这里可以将 sendFunc 存储在静态成员变量中，供后续指令发送时调用
    BenMoMotor::_sendFunction = sendFunc;
}

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
 * @brief 通用包构建器 (10字节固定长度)：直接操作传入的数组指针
 */
void BenMoMotor::buildBasicPacket(uint8_t* buf, uint8_t reg, uint16_t val, uint8_t d6, uint8_t d7) {
    if(buf == nullptr|| BenMoMotor::_sendFunction == nullptr) return; // 安全检查，防止空指针访问
    memset(buf, 0, 10);      // 先清空数组
    buf[0] = _id;            // ID
    buf[1] = reg;            // 指令标识符
    buf[2] = (val >> 8);     // 数据高位
    buf[3] = (val & 0xFF);   // 数据低位
    buf[4] = 0;             // 保留
    buf[5] = 0;             // 保留
    buf[6] = d6;             // 加速时间
    buf[7] = d7;             // 刹车位
    buf[9] = calculateCRC8(buf, 9); // 计算前9位的CRC填入第10位
    // 通过注册的发送函数发送指令包
    BenMoMotor::_sendFunction(_port_num, buf, MOTOR_PACKET_SIZE);
}

/**
 * @brief 切换运行模式 (对应手册 P10)
 * 模式值: 0x00 开环, 0x01 电流环, 0x02 速度环, 0x03 位置环
 */
void BenMoMotor::genModeCmd(uint8_t mode_val) {
    buildBasicPacket(_out_packet, 0xA0, (uint16_t)mode_val << 8);
}

/**
 * @brief 使能/失能 (对应手册 P10 模式值 0x08/0x09)
 * 注意：使能/失能指令与切换模式指令共用寄存器 0xA0，DATA[2] 的值区分功能：
 * - 切换模式: DATA[2] = 0x00~0x03
 * - 使能/失能: DATA[2] = 0x08 (使能) 或 0x09 (失能)
 */
void BenMoMotor::genEnableCmd(bool enable) {
    buildBasicPacket(_out_packet, 0xA0, (enable ? 0x08 : 0x09) << 8);
}

/**
 * @brief 速度控制 (对应手册 P8)
 * target_rpm: 目标转速(RPM)。实际写入值 = target_rpm * 10
 * accel_time: 加速时间 (ms/1rpm), 0表示最快。默认1。
 */
void BenMoMotor::genSpeedCtrl(float target_rpm, uint8_t accel_time, bool brake) {
    // 转换规则：写入值 = 实际转速 * 10
    int16_t val = (int16_t)(target_rpm * 10.0f);
    buildBasicPacket(_out_packet, 0x64, (uint16_t)val, accel_time, (brake ? 0xFF : 0x00));
}

/**
 * @brief 电流/力矩控制 (对应手册 P8)
 * current_raw: -32767 ~ 32767 (对应 -4A 到 4A)
 */
void BenMoMotor::genCurrentCtrl(int16_t current_raw) {
    buildBasicPacket(_out_packet, 0x64, (uint16_t)current_raw);
}

/**
 * @brief 位置控制 (对应手册 P8)
 * position_value: 0~32767 对应 0~360°
 */
void BenMoMotor::genPositionCtrl(uint16_t position_value) {
    buildBasicPacket(_out_packet, 0x64, position_value);
}

/**
 * @brief 查询额外反馈 (对应手册 P9)
 * 该指令请求电机返回里程、位置和当前模式等高级状态信息，反馈ID为 0x75/0x76。
 */
void BenMoMotor::genQueryExtraCmd() {
    buildBasicPacket(_out_packet, 0x74, 0x0000);
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
bool BenMoMotor::parseDriveFeedback(const uint8_t* buf) {
    //考虑加入临界区包护，防止解析过程中被新的数据覆盖
    memcpy(_in_packet, buf, MOTOR_PACKET_SIZE); // 先将输入数据复制到内部缓冲区，方便调试和后续处理

    if (_in_packet[0] != _id || _in_packet[1] != 0x65) return false;
    if (calculateCRC8(_in_packet, 9) != _in_packet[9]) return false;

    __drive_status.speed = (int16_t)((_in_packet[2] << 8) | _in_packet[3]);   // 需要除以10得到真实RPM
    __drive_status.current = (int16_t)((_in_packet[4] << 8) | _in_packet[5]);
    __drive_status.temp = _in_packet[7];
    __drive_status.fault_code = _in_packet[8];
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
bool BenMoMotor::parseExtraFeedback(const uint8_t* buf) {
    //考虑加入临界区包护，防止解析过程中被新的数据覆盖
    memcpy(_in_packet, buf, MOTOR_PACKET_SIZE); // 先将输入数据复制到内部缓冲区，方便调试和后续处理

    // 0x75 为请求回复，0x76 为模式反馈
    if (_in_packet[0] != _id || (_in_packet[1] != 0x75 && _in_packet[1] != 0x76)) return false;
    if (calculateCRC8(_in_packet, 9) != _in_packet[9]) return false;

    if (_in_packet[1] == 0x75) {
        // 里程 4 字节
        __extra_status.mileage = (int32_t)((_in_packet[2] << 24) | (_in_packet[3] << 16) | (_in_packet[4] << 8) | _in_packet[5]);
        // 位置 2 字节
        __extra_status.position = (uint16_t)((_in_packet[6] << 8) | _in_packet[7]);
    } else {
        // 0x76 模式反馈包
        __extra_status.mode = _in_packet[2];
    }
    return true;
}