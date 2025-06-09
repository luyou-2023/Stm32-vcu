/*
 *
 * Copyright (C) 2020 Johannes Huebner <dev@johanneshuebner.com>
 *                      Damien Maguire <info@evbmw.com>
 * 这里是版权信息和作者联系方式
 * “是的，我现在真的在写软件了……快跑……快跑吧……”
 *
 * 该程序是自由软件：您可以根据GNU通用公共许可证（GPL）第3版或（您选择的）更高版本自由地重新发布和/或修改本程序。
 *
 * 该程序发布的目的是希望它有用，但不保证任何形式的担保，包括对适销性或适用于特定目的的隐含保证。
 *
 * 您应当已收到一份GNU通用公共许可证的副本。如果没有，请访问 http://www.gnu.org/licenses/ 。
 *
 * 本文件基于V2.02版本重新实现。更多信息见：https://openinverter.org/wiki/CAN_communication
 */

#include "Can_OI.h"
#include "my_fp.h"
#include "my_math.h"
#include "stm32_can.h"
#include "params.h"

// 静态变量定义
uint8_t Can_OI::run100ms = 0;       // 100ms定时任务标志
uint32_t Can_OI::lastRecv = 0;      // 最近一次CAN接收时间戳
uint16_t Can_OI::voltage;            // 逆变器电压
int16_t Can_OI::speed;               // 电机转速
bool Can_OI::error=false;            // 错误状态标志
int16_t Can_OI::inv_temp;            // 逆变器温度
int16_t Can_OI::motor_temp;          // 电机温度
int16_t Can_OI::final_torque_request; // 最终扭矩请求值
static bool statusInv = 0;           // 逆变器状态标志，静态局部变量
uint8_t Inv_Opmode=0;                // 逆变器工作模式
int opmode;                         // 当前操作模式
uint16_t InvStartTimeout=0;          // 逆变器启动超时计数器

// 设置CAN接口并注册关心的消息ID
void Can_OI::SetCanInterface(CanHardware* c)
{
   can = c;

   // 注册不同的消息ID，这些消息包含电机转速、电压、温度和工作模式
   can->RegisterUserMessage(0x190); // 电机转速消息，ID为0x190，解码位400
   can->RegisterUserMessage(0x19A); // 温度消息，ID为0x19A，解码位410
   can->RegisterUserMessage(0x1A4); // 电压消息，ID为0x1A4，解码位420
   can->RegisterUserMessage(0x1AE); // 工作模式消息，ID为0x1AE，解码位430
}

// 解码接收到的CAN消息，根据ID解析不同数据
void Can_OI::DecodeCAN(int id, uint32_t data[2])
{
   // CAN消息格式说明：
   // 0x1A4消息中，位0-15为逆变器电压*10
   // 0x190消息中，位0-15为电机转速
   // 0x19A消息中，位0-15为散热器温度*10

   uint8_t* bytes = (uint8_t*)data; // 将32位数据转换为字节数组便于处理

   if (id == 0x1A4) // 逆变器电压消息处理
   {
      // 电压是低字节在前，高字节在后，转换为实际电压值
      voltage = ((bytes[1] << 8) | (bytes[0])) / 10;
   }
   else if (id == 0x190) // 电机转速消息处理
   {
      speed = ((bytes[1] << 8) | (bytes[0]));
   }
   else if (id == 0x19A) // 逆变器散热器温度消息处理
   {
      inv_temp = ((bytes[1] << 8) | (bytes[0])) / 10; // 逆变器温度
      motor_temp = 0; // 电机温度暂时未使用，设为0
   }
   else if (id == 0x1AE) // 逆变器工作模式消息处理
   {
      Inv_Opmode = bytes[0]; // 工作模式字节
      // 具体模式说明：
      // 0=关闭, 1=运行, 2=手动运行, 3=Boost模式, 4=Buck模式, 5=正弦波, 6=交流加热
   }
}

// 设置请求的扭矩百分比
void Can_OI::SetTorque(float torquePercent)
{
   // 将扭矩百分比转换成整型值（放大10倍），以便发送
   final_torque_request = torquePercent * 10;

   // 设置参数系统中的扭矩值，供web接口等使用
   Param::SetInt(Param::torque, final_torque_request);

   // 读取当前操作模式
   int opmode = Param::GetInt(Param::opmode);

   uint8_t tempIO = 0; // 用于存放方向和状态的IO位

   // 只有在运行模式下，才发送前进和倒退方向信息
   if (Param::GetBool(Param::din_forward) && opmode == MOD_RUN) tempIO += 8;
   if (Param::GetBool(Param::din_reverse) && opmode == MOD_RUN) tempIO += 16;
   if (Param::GetBool(Param::din_brake)) tempIO += 4;
   // if(Param::GetBool(Param::din_start)) tempIO+=2; // 启动信号，暂时注释

   // 进入运行模式时，启动信号保持3秒
   if (opmode == MOD_RUN && InvStartTimeout != 0)
   {
      InvStartTimeout--;
      tempIO += 2; // 启动信号有效
   }

   // 退出运行模式，重置启动超时计数
   if (opmode == MOD_OFF)
   {
      InvStartTimeout = 300;
   }

   // 构造CAN数据包内容
   uint32_t data[2];
   uint32_t pot = Param::GetInt(Param::pot) & 0xFFF;       // 油门信号，占12位
   uint32_t pot2 = Param::GetInt(Param::pot2) & 0xFFF;     // 第二油门信号，占12位
   uint32_t canio = tempIO & 0x3F;                         // IO信号占6位
   uint32_t ctr = Param::GetInt(Param::canctr) & 0x3;      // 计数器，占2位
   uint32_t cruise = Param::GetInt(Param::cruisespeed) & 0x3FFF; // 巡航速度，占14位
   uint32_t regen = 0x00;                                  // 再生制动，目前为0

   // 组合数据位，符合CAN协议要求
   data[0] = pot | (pot2 << 12) | (canio << 24) | (ctr << 30);
   data[1] = cruise | (ctr << 14) | (regen << 16);

   // 计算CRC校验码，并放入数据末尾
   crc_reset();
   uint32_t crc = crc_calculate_block(data, 2) & 0xFF;
   data[1] |= crc << 24;

   // 发送CAN消息，ID为0x3F
   can->Send(0x3F, data);
}

// 获取逆变器当前状态，1为运行，0为关闭
int Can_OI::GetInverterState()
{
   if (Inv_Opmode == 0) statusInv = 0;
   if (Inv_Opmode == 1) statusInv = 1;

   // 如果操作模式为关闭，强制状态为关闭
   if (opmode == MOD_OFF) statusInv = 0;

   return statusInv;
}

// 100ms定时任务，定时更新和清理变量
void Can_OI::Task100Ms()
{
   // 读取当前操作模式
   opmode = Param::GetInt(Param::opmode);

   if (opmode == MOD_OFF)
   {
      // 当处于关闭状态时，清零所有参数，确保下次启动时显示真实值
      voltage = 0;
      speed = 0;
      inv_temp = 0;
      motor_temp = 0;
      Inv_Opmode = 0;
      final_torque_request = 0;
   }
}
