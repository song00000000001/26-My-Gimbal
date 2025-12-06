/**
 * @file protocol.cpp
 * @author Meng Yang (2895422061@qq.com)
 * @brief 实现第三代调参板的通信协议
 * @version 0.1
 * @date 2024-10-08
 * 
 * @copyright Copyright (c) 2024 - ~  SCUT-RobotLab Development team
 *  ******************************************************************************
 * @attention
 * *
 * * if you had modified this file, please make sure your code does not have many
 * * bugs, update the version Number, write dowm your name and the date, the most
 * * important is make sure the users will have clear and definite understanding
 * * through your new brief.
 * *
 * * <h2><center>&copy; Copyright (c) 2024 - ~, SCUT-RobotLab Development Team.
 * * All rights reserved.</center></h2>
 * ******************************************************************************
 * 
 */
#include "internal.h"
#include "global_data.h"
#include "protocol.h"

#pragma pack(1)
struct UniversalHeaderStructdef
{
  uint8_t SOF;
  uint8_t CmdID;
}UniversalHeader;
struct LaunchSequceChangeStructdef
{
  uint8_t SOF;
  uint8_t CmdID; //修改发射序列，命令码0x01
  uint8_t seq_indx:4; //发射序列下标
  uint8_t param_indx:4; //参数下标
  uint8_t CRC8;
}LaunchSequceChangePack;
struct ParamChangeStructdef
{
  uint8_t SOF;
  uint8_t CmdID; //改参,命令码0x02
  uint8_t param_indx; //参数下标
  double outpost_launch;
  double outpost_yaw;
  double base_launch;
  double base_yaw;
  uint8_t CRC8;
}ParamChangePack;
struct MisCmdStructdef
{
  uint8_t SOF;
  uint8_t CmdID; //命令码
  uint8_t payload;  //指令
  uint8_t CRC8;
}MisCmdPack;
struct DownLinkStructdef
{
  uint8_t SOF;
  uint8_t CmdID;
  uint8_t cur_num;
  uint8_t launcher_state:4;
  uint8_t ref_state:1;
  uint8_t remote_state:1;
  uint8_t target:2;
  uint8_t CRC8;
}DownLinkPack;
#pragma pack()

Missle_State_t state = DEINIT;
uint8_t DartDataSlot[5]={0,1,2,3,4}; // 发射数据选择
DartDataStructdef DartsData[MAX_DART_DATAPOOL_SIZE]; //发射数据


void SendHeartBeat();
void Launch_Callback()
{
    //Launch.Igniter_On();
}
void packDecoder(uint8_t * _addr,uint8_t len)
{
  if(_addr[0] != 0xa5) return;
  if(_addr[1] == 0x01) //命令码0x01，修改发射序列
  {
    if(len != sizeof(LaunchSequceChangePack)) // 检查数据包长度
      return;
    memcpy(&LaunchSequceChangePack,_addr,len);
    if(std_lib::CRC8(&LaunchSequceChangePack,sizeof(LaunchSequceChangePack)) != 0) //检查CRC校验
      return;
    DartDataSlot[LaunchSequceChangePack.seq_indx + 1] = LaunchSequceChangePack.param_indx;
  }
  if(_addr[1] == 0x02) //命令码0x02，改参
  {
    if(len != sizeof(ParamChangePack)) // 检查数据包长度
      return;
    memcpy(&ParamChangePack,_addr,len);
    if(std_lib::CRC8(&ParamChangePack,sizeof(ParamChangePack)) != 0) //检查CRC校验
      return;
    DartsData[ParamChangePack.param_indx].Ignitergoal[Outpost] =ParamChangePack.outpost_launch;
    DartsData[ParamChangePack.param_indx].Ignitergoal[Base] =ParamChangePack.base_launch;
    DartsData[ParamChangePack.param_indx].YawCorrectionAngle[Outpost] = ParamChangePack.outpost_yaw;
    DartsData[ParamChangePack.param_indx].YawCorrectionAngle[Base] = ParamChangePack.base_yaw;
  }
  if(_addr[1] == 0x03) //命令码0x03 修改打击目标
  {
    if(len != sizeof(MisCmdPack)) // 检查数据包长度
      return;
    memcpy(&MisCmdPack,_addr,len);
    if(std_lib::CRC8(&MisCmdPack,sizeof(MisCmdPack)) != 0) //检查CRC校验
      return;
    HitTarget = (DartAimEnumdef)MisCmdPack.payload;
  }
  if(_addr[1] == 0x04) //命令码0x04 发射！
  {
    if(len != sizeof(MisCmdPack)) // 检查数据包长度
      return;
    memcpy(&MisCmdPack,_addr,len);
    if(std_lib::CRC8(&MisCmdPack,sizeof(MisCmdPack)) != 0) //检查CRC校验
      return;
    Launch_Callback(); //射
  }
  if(_addr[1] == 0x05) //命令码0x05查询精神状况
  {
    if(len != sizeof(MisCmdPack)) // 检查数据包长度
      return;
    memcpy(&MisCmdPack,_addr,len);
    if(std_lib::CRC8(&MisCmdPack,sizeof(MisCmdPack)) != 0) //检查CRC校验
      return;
    if(MisCmdPack.payload != 0xff)
      return;
    SendHeartBeat();
  }
}

/**
 * @brief 飞镖调参版数据查询接口
 *
 * @return uint8_t
 */

bool Is_Launching()
{
	return (state == WAIT_SHOOT || state == PULL);
}

void SendHeartBeat()
{
  DownLinkPack.SOF = 0xA5;
  DownLinkPack.CmdID = 0x11;
  //DownLinkPack.cur_num = CurrentCnt - 1;
 if(Is_Launching())
 {
   DownLinkPack.launcher_state = 4; //回传镖架状态
 }
 else
 {
   DownLinkPack.launcher_state = 1;
 }
 DownLinkPack.ref_state = Referee.status;
  DownLinkPack.remote_state = DR16.GetStatus() == DR16_ESTABLISHED;
  DownLinkPack.target = HitTarget;
  USART_COB tmp;
  tmp.port_num = 3;
  tmp.address = (uint8_t *)&DownLinkPack;
  tmp.len = sizeof(DownLinkPack);
  SRML_UART_Transmit_DMA(&tmp);
}
