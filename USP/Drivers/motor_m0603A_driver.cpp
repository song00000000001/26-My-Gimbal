#include "motor_m0603A_driver.h"
#include "robot_config.h"

// 静态成员变量定义和初始化
bool (*BenMoMotor::_sendFunction)(uint8_t, const uint8_t*, uint8_t) = nullptr;

BenMoMotor::BenMoMotor(uint8_t motor_id) : _id(motor_id) {}

/**
 * @brief 注册串口发送函数
 * @param sendFunc 用户提供的发送函数指针，函数签名为 bool sendFunc(uint8_t port_id, const uint8_t* data, uint8_t len)，用于发送生成的指令包
 * 该函数需要在使用驱动前调用一次，注册后驱动内部会使用该函数发送指令包，用户无需关心通信细节，只需提供一个符合签名的发送函数即可。
 */
void BenMoMotor::registerSendFunction(bool (*sendFunc)(uint8_t port_id,const uint8_t* data, uint8_t len)) {
    if(sendFunc == nullptr) return; // 安全检查，避免注册空指针
    // 这里将 sendFunc 存储在静态成员变量中，供后续指令发送时调用
    BenMoMotor::_sendFunction = sendFunc;
}

/**
 * @brief CRC-8/MAXIM 校验实现 (多项式 x8 + x5 + x4 + 1)
 */
uint8_t BenMoMotor::calculateCRC8(const uint8_t* data, uint8_t len) {
    if(data == nullptr || len == 0) return 0; // 安全检查
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
 * @param buf 用于构建指令包的缓冲区，必须至少10字节长度
 * @param reg 寄存器地址 (指令ID)
 * @param val 16位数据值，分为高8位和低8位存储在 data_h 和 data_l
 * @param d6 扩展数据6 (如加速时间等)，默认为0
 * @param d7 扩展数据7 (如刹车位等)，默认为0
 * @details 该函数负责填充指令包的各个字段，并计算CRC校验位，最后通过注册的发送函数发送指令包。
 * @details 前三个参数是必需的，后两个参数是可选的，默认0。
 */
void BenMoMotor::buildBasicPacket(uint8_t* buf, uint8_t reg, uint16_t val, uint8_t d6, uint8_t d7) {
    if(buf == nullptr || BenMoMotor::_sendFunction == nullptr) return; // 安全检查

    //内部延时，方便使用。
    // static TickType_t xLastWakeTime_t;
    // xLastWakeTime_t = xTaskGetTickCount();
    // const TickType_t xFrequency = pdMS_TO_TICKS(motor_comm_delay_ms);
    // vTaskDelayUntil(&xLastWakeTime_t, xFrequency);

    BenMoMotorPacket* pkt = reinterpret_cast<BenMoMotorPacket*>(buf);
    /**
     * c++语法知识点补充：
     * reinterpret_cast 用于在不同类型之间进行强制转换，这里将字节数组解释为 BenMoMotorPacket 结构体，方便按字段赋值。
      *其实不写 reinterpret_cast 直接用 buf->id 也是可以的，因为 buf 是 uint8_t* 类型，编译器会根据访问的字段自动计算偏移。
      *但使用 reinterpret_cast 可以更清晰地表达意图，明确这是在处理一个特定格式的数据包。
      */
    memset(pkt, 0, sizeof(BenMoMotorPacket));

    pkt->id   = _id;
    pkt->reg  = reg;
    pkt->data_h = (uint8_t)(val >> 8);
    pkt->data_l = (uint8_t)(val & 0xFF);
    pkt->d6   = d6;
    pkt->d7   = d7;
    pkt->crc  = calculateCRC8(buf, 9);

    // 通过注册的发送函数发送指令包
    BenMoMotor::_sendFunction(_port_num, buf, MOTOR_PACKET_SIZE);
}

/**
 * @brief 切换运行模式 (对应手册 P10)
 * 模式值: 0x00 开环, 0x01 电流环, 0x02 速度环, 0x03 位置环
 */
void BenMoMotor::sendModeCmd(MotorMode mode_val) {
    _current_mode = mode_val;
    // 使用 static_cast 将强类型枚举转为底层数字
    buildBasicPacket(_out_packet, 0xA0, static_cast<uint16_t>(_current_mode) << 8);
}

/**
 * @brief 使能/失能 (对应手册 P10 模式值 0x08/0x09)
 * 注意：使能/失能指令与切换模式指令共用寄存器 0xA0，DATA[2] 的值区分功能：
 * - 切换模式: DATA[2] = 0x00~0x03
 * - 使能/失能: DATA[2] = 0x08 (使能) 或 0x09 (失能)
 */
void BenMoMotor::sendEnableCmd(bool enable) {
    buildBasicPacket(_out_packet, 0xA0, (enable ? 0x08 : 0x09) << 8);
}

/**
 * @brief 速度控制 (对应手册 P8)
 * @param target_rpm: 目标转速(RPM)。范围 -380.0 ~ 380.0 RPM，负值表示反转。
 * @details 实际写入值 = target_rpm * 10
 * @param accel_time: 加速时间 (ms/1rpm), 0表示最快。默认1。
 * @param brake: 是否刹车 (仅速度环有效)
 */
void BenMoMotor::sendSpeedCtrl(float target_rpm, uint8_t accel_time, bool brake) {
    if(target_rpm < -380.0f) target_rpm = -380.0f;
    if(target_rpm > 380.0f) target_rpm = 380.0f;
    // 转换规则：写入值 = 实际转速 * 10
    int16_t val = (int16_t)(target_rpm * 10.0f);
    buildBasicPacket(_out_packet, 0x64, (uint16_t)val, accel_time, (brake ? 0xFF : 0x00));
}

/**
 * @brief 电流/力矩控制 (对应手册 P8)
 * @param current_raw: 目标电流值，单位为A，范围 -4A ~ 4A。
 * @details 实际写入值 = (current_raw / 4.0) * 32767
 */
void BenMoMotor::sendCurrentCtrl(float current_raw) {
    if(current_raw < -4.0f) current_raw = -4.0f;
    if(current_raw > 4.0f) current_raw = 4.0f;
    uint16_t val = (uint16_t)(current_raw / 4.0f * 32767.0f);
    buildBasicPacket(_out_packet, 0x64, val, 0, 0);
}

/**
 * @brief 位置控制 (对应手册 P8)
 * @param position_value 目标位置，单位为度，范围 0~360°。
 * @details 实际写入值 = (position_value / 360) * 32767
 */
void BenMoMotor::sendPositionCtrl(float position_value, uint8_t accel_time) {
    if(position_value < 0.0f) position_value = 0.0f;
    if(position_value > 360.0f) position_value = 360.0f;
    uint16_t val = (uint16_t)((position_value / 360.0f) * 32767.0f);
    buildBasicPacket(_out_packet, 0x64, val, accel_time, 0);
}

/**
 * @brief 查询额外反馈 (对应手册 P9)
 * 该指令请求电机返回里程、位置和当前模式等高级状态信息，反馈ID为 0x75/0x76。
 * @todo 有问题，额外数据一直是0。需要排查是发送指令有问题还是解析有问题，或者电机固件不支持这个功能。
 * 
 */
void BenMoMotor::sendQueryExtraCmd() {
    buildBasicPacket(_out_packet, 0x74, 0);
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
    if (buf == nullptr) return false;
    
    const BenMoMotorPacket* pkt = reinterpret_cast<const BenMoMotorPacket*>(buf);
    /*
    reinterpret_cast 将底层字节流强行解释为特定结构。这在处理网络协议或硬件驱动时非常高效。
    */

    if (pkt->id != _id || pkt->reg != 0x65) return false;
    if (calculateCRC8(buf, 9) != pkt->crc) return false;

    // 复制一份以便后续处理
    memcpy(_in_packet, buf, MOTOR_PACKET_SIZE);

    
    __drive_status.speed   = (int16_t)((pkt->data_h << 8) | pkt->data_l); 
    __drive_status.current = (int16_t)((pkt->reserved1 << 8) | pkt->reserved2); // 手册中 0x65 反馈的 Byte 4-5
    __drive_status.temp    = pkt->d7;
    __drive_status.fault_code = pkt->reserved3;
    
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
    if (buf == nullptr) return false;
    
    const BenMoMotorPacket* pkt = reinterpret_cast<const BenMoMotorPacket*>(buf);

    // 0x75 为请求回复，0x76 为模式反馈
    if (pkt->id != _id || (pkt->reg != 0x75 && pkt->reg != 0x76)) return false;
    if (calculateCRC8(buf, 9) != pkt->crc) return false;

    memcpy(_in_packet, buf, MOTOR_PACKET_SIZE);

    if (pkt->reg == 0x75) {
        // 里程 4 字节 (Byte 2-5)
        __extra_status.mileage = (int32_t)((pkt->data_h << 24) | (pkt->data_l << 16) | (pkt->reserved1 << 8) | pkt->reserved2);
        // 位置 2 字节 (Byte 6-7)
        __extra_status.position = (uint16_t)((pkt->d6 << 8) | pkt->d7);
    } else {
        // 0x76 模式反馈包 (Byte 2)
        __extra_status.mode = pkt->data_h;
    }
    return true;
}


/**
 * @brief 解析电机模式反馈 (对应手册 P9 反馈ID 0xA1)
 * 反馈数据格式 (10字节):
    * Byte 0: 电机ID
    * Byte 1: 反馈ID (0xA1)
    * Byte 2: 电机模式值
    * 模式值:
        0x00:设定为开环
        0x01:设定为电流环
        0x02:设定为速度环
        0x08:电机使能
        0x09:电机失能
        0x0A:电机后转150±10°
        0x10:开启通讯断联功能，超过3S没有接收到信息会停止动作，维持上一个模式
        0x11:关闭通讯断联功能
 */
bool BenMoMotor::parseModeFeedback(const uint8_t* buf) {
    if (buf == nullptr) return false;
    
    const BenMoMotorPacket* pkt = reinterpret_cast<const BenMoMotorPacket*>(buf);

    // 0x75 为请求回复，0x76 为模式反馈
    if (pkt->id != _id || (pkt->reg != 0xA1)) return false;
    if (calculateCRC8(buf, 9) != pkt->crc) return false;

    __extra_status.mode = pkt->data_h; // 模式值在 Byte 2 (data_h)

    return true;
}