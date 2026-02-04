#include "srml_config.h"

#if 1

#include "FS_I6X.h"
using namespace std_lib;

/**
 * @brief 将拨杆通道值处理成拨杆状态
 * 
 * @param ch_val 拨杆通道值
 * @return SW_Status_Typedef 拨杆状态
 */
SW_Status_Typedef FS_I6X_Classdef::SW_Process(uint16_t ch_val)
{
    if(ch_val < 300)
        return SW_UP;
    else if(ch_val > 1750)
        return SW_DOWN;
    else
        return SW_MID;
}

/**
 * @brief 连接检测函数
 * 
 * @param current_check_time 当前系统时间，单位为ms
 */
void FS_I6X_Classdef::Check_Link(uint32_t current_check_time)
{
    /*开始检测*/
    if (DataPack.HEAD == 0x0f && DataPack.END == 0)
    {
        Status = ESTABLISHED;
        last_recv_time = current_check_time;
    }
    else if (current_check_time - last_recv_time > LOST_THRESHOLD)
    {
        Status = LOST;
    }
    DataPack.HEAD = DataPack.END = 0xA5;
}

/**
 * @brief 数据处理函数
 * 
 */
void FS_I6X_Classdef::DataProcess()
{
    // 要摇杆映射为-1.1至1.1，然后限幅为-1至1，避免部分摇杆无法到达1的情况
    RX_Norm = DeadZone_Process((DataPack.ch1 - 1024) / 784.0f * 1.1f, -DeadZone, DeadZone);
    RY_Norm = DeadZone_Process((DataPack.ch2 - 1024) / 784.0f * 1.1f, -DeadZone, DeadZone);
    LY_Norm = DeadZone_Process((DataPack.ch3 - 1024) / 784.0f * 1.1f, -DeadZone, DeadZone);
    LX_Norm = DeadZone_Process((DataPack.ch4 - 1024) / 784.0f * 1.1f, -DeadZone, DeadZone);
    VRA_Norm = DeadZone_Process((DataPack.ch5 - 1024) / 784.0f * 1.1f, -DeadZone, DeadZone);
    VRB_Norm = DeadZone_Process((DataPack.ch6 - 1024) / 784.0f * 1.1f, -DeadZone, DeadZone);

    RX_Norm = constrain(RX_Norm, -1.f, 1.f);
    RY_Norm = constrain(RY_Norm, -1.f, 1.f);
    LY_Norm = constrain(LY_Norm, -1.f, 1.f);
    LX_Norm = constrain(LX_Norm, -1.f, 1.f);
    VRA_Norm = constrain(VRA_Norm, -1.f, 1.f);
    VRB_Norm = constrain(VRB_Norm, -1.f, 1.f);

    SWA = SW_Process(DataPack.ch7);
    SWB = SW_Process(DataPack.ch8);
    SWC = SW_Process(DataPack.ch9);
    SWD = SW_Process(DataPack.ch10);
}

#endif // !USE_SRML_FS_I6X