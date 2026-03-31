#pragma once

#include <stdint.h>
#include <stdbool.h>
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

/*todo
song
打算重构驱动代码，上层不需要知道通信细节，直接调用接口。比如设置目标速度，目标角度，模式，限幅等。不然让上层打包太麻烦。
然后提供一个内部函数接口，上层注册好串口发送函数，驱动内部根据当前状态和目标状态计算控制输出，并通过注册的发送函数发送指令包。
这样上层只需要关心控制逻辑，不需要关心通信细节，驱动内部封装好通信协议和数据解析。后续可以考虑增加更复杂的滤波算法，或者增加更多的控制模式（比如位置环）。
*/

class BenMoMotor {
public:
    /**
     * @param motor_id 电机ID (通常为 1 或 2)
     */
    BenMoMotor(uint8_t motor_id);

    /**
     * @brief 注册发送函数指针，供驱动内部调用发送指令包
     * @param sendFunc 函数指针，函数签名为 bool func(uint8_t port_id,const uint8_t* data, uint8_t len)，返回值表示发送是否成功
     * 该函数需要在使用驱动前调用一次，注册后驱动内部会使用该函数发送指令包，用户无需关心通信细节，只需提供一个符合签名的发送函数即可。
     * c语音知识点补充：函数指针是一种特殊的指针变量，用于存储函数的地址。通过函数指针，程序可以动态调用不同的函数，实现更灵活的设计。
        * 类中使用static成员变量存储函数指针，意味着这个函数指针是类级别的，所有实例共享同一个发送函数。
        * 类中的成员函数使用命名空间限定符BenMoMotor::来修饰，表明调用的是BenMoMotor类的所有实例共有的成员函数。
        * 如果用this->来调用成员函数，则会调用当前对象的成员函数，而不是类级别的静态成员函数，可能会因此导致编译错误。
     * 
     */
    static void registerSendFunction(bool (*sendFunc)(uint8_t port_id,const uint8_t* data, uint8_t len));
    /**
     * @brief 设置发送使用的串口ID
     * @param port_id 串口ID，例如 1 表示 USART1
     */
    void setPortNum(uint8_t port_id) { _port_num = port_id; }
    // ==================== [发送指令生成] ====================

    // 生成切换模式指令 (0xA0)
    void genModeCmd(uint8_t mode_val);

    // 生成使能/失能指令 (0xA0 0x08/0x09)
    void genEnableCmd(bool enable);

    // 生成速度控制指令 (0x64)
    // target_rpm: 目标转速(0.1RPM)。实际写入值 = target_rpm * 10
    // accel_time: 加速时间 (ms/1rpm), 0表示最快。默认1。
    // brake: 是否刹车 (仅速度环有效)
    void genSpeedCtrl(float target_rpm, uint8_t accel_time = 0, bool brake = false);

    // 生成电流/力矩控制指令 (0x64)
    // current_raw: -32767 ~ 32767 (对应 -4A 到 4A)
    void genCurrentCtrl(int16_t current_raw);

    // 生成位置控制指令 (0x64)
    void genPositionCtrl(uint16_t position_value);

    // 生成请求其他反馈指令 (0x74) - 获取里程和精确位置
    void genQueryExtraCmd();

    // ==================== [数据解析逻辑] ====================

    // 解析 ID 0x65 的反馈数据 (速度、电流、温度)
    bool parseDriveFeedback(const uint8_t* buf);

    // 解析 ID 0x75/0x76 的反馈数据 (里程、位置、模式)
    bool parseExtraFeedback(const uint8_t* buf);

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
    uint8_t _port_num; // 发送使用的串口ID
    MotorDriveStatus __drive_status; // 上次的常规反馈状态
    MotorExtraStatus __extra_status; // 上次的额外反馈状态
    uint8_t _out_packet[MOTOR_PACKET_SIZE]; // 用于生成指令的缓冲区
    uint8_t _in_packet[MOTOR_PACKET_SIZE];  // 用于解析反馈的缓冲区
    // 内部通用的填包逻辑
    void buildBasicPacket(uint8_t* buf, uint8_t reg, uint16_t val, uint8_t d6 = 0, uint8_t d7 = 0);
    // 存储用户注册的发送函数指针
    static bool (*_sendFunction)(uint8_t port_id,const uint8_t* data, uint8_t len);
};


