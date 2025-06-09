/*
 * 这个模块用于接收来自SimpBMS的CAN消息，并根据接收到的数据
 * 更新BMS的最小/最大电压和温度参数（BMS_MinV, BMS_MaxV, BMS_MinT, BMS_MaxT）。
 * 同时实现了一个超时机制，用于判断BMS是否还在发送数据，
 * 以确保充电过程的安全。
 */

#include "stm32_vcu.h"

// 设置CAN接口，并注册感兴趣的CAN消息ID
void SimpBMS::SetCanInterface(CanHardware* c)
{
   can = c;
   // 注册CAN消息ID 0x373，用于电压和温度数据
   can->RegisterUserMessage(0x373);
   // 注册CAN消息ID 0x351，用于充电电流限制数据
   can->RegisterUserMessage(0x351);
}

// 判断BMS数据是否有效（即是否收到BMS数据且未超时）
bool SimpBMS::BMSDataValid() {
   if(timeoutCounter < 1)
       return false; // 超时计数为0，表示没有接收到有效数据
   return true;
}

// 判断是否允许充电
bool SimpBMS::ChargeAllowed()
{
   // 如果BMS数据无效，不允许充电
   if(!BMSDataValid()) return false;

   // 电压超出允许范围，不允许充电
   if(maxCellV > Param::GetFloat(Param::BMS_VmaxLimit)) return false;
   if(minCellV < Param::GetFloat(Param::BMS_VminLimit)) return false;

   // 温度超出允许范围，不允许充电
   if(maxTempC > Param::GetFloat(Param::BMS_TmaxLimit)) return false;
   if(minTempC < Param::GetFloat(Param::BMS_TminLimit)) return false;

   // 充电电流限制小于0.5（单位应为mA），不允许充电
   if(chargeCurrentLimit < 0.5) return false;

   // 以上条件都满足，允许充电
   return true;
}

// 获取BMS允许的最大充电电流（单位A）
float SimpBMS::MaxChargeCurrent()
{
   if(!ChargeAllowed()) return 0;
   return chargeCurrentLimit / 1000.0; // 转换mA到A
}

// 解析接收到的CAN消息数据
void SimpBMS::DecodeCAN(int id, uint8_t *data)
{
   if (id == 0x373)
   {
      // 电压和温度数据格式解析（两个字节低字节在前）
      int minCell = data[0] | (data[1] << 8);
      int maxCell = data[2] | (data[3] << 8);
      int minTemp = data[4] | (data[5] << 8);
      int maxTemp = data[6] | (data[7] << 8);

      // 转换数据单位
      minCellV = minCell / 1000.0;          // 电压单位mV转V
      maxCellV = maxCell / 1000.0;
      minTempC = minTemp - 273;             // 开尔文转摄氏度
      maxTempC = maxTemp - 273;

      // 重置超时计数器，乘以10是为了对应100ms任务周期
      timeoutCounter = Param::GetInt(Param::BMS_Timeout) * 10;
   }
   else if (id == 0x351)
   {
      // 充电电流限制数据解析，两个字节低字节在前
      chargeCurrentLimit = data[2] | (data[3] << 8);
   }
}

// 每100ms执行一次的任务，更新超时计数器和参数
void SimpBMS::Task100Ms() {
   // 超时计数器递减
   if(timeoutCounter > 0) timeoutCounter--;

   // 更新充电限制参数
   Param::SetInt(Param::BMS_ChargeLim, MaxChargeCurrent());

   if(BMSDataValid()) {
      // BMS数据有效，更新电压和温度参数
      Param::SetFloat(Param::BMS_Vmin, minCellV);
      Param::SetFloat(Param::BMS_Vmax, maxCellV);
      Param::SetFloat(Param::BMS_Tmin, minTempC);
      Param::SetFloat(Param::BMS_Tmax, maxTempC);
   }
   else
   {
      // BMS数据无效，清零参数
      Param::SetFloat(Param::BMS_Vmin, 0);
      Param::SetFloat(Param::BMS_Vmax, 0);
      Param::SetFloat(Param::BMS_Tmin, 0);
      Param::SetFloat(Param::BMS_Tmax, 0);
   }
}
