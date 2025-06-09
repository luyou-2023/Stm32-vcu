#include <charger.h>

// chargerClass 类的静态成员，表示高压请求状态，默认false（未请求）
bool chargerClass::HVreq = false;

// 静态计数器，用于消息计数，范围0~15
static uint8_t counter_109 = 0;

/**
 * 处理CAN消息ID为0x108的高压请求消息
 * 参数 data 是一个包含两个32位数据的数组
 *
 * 处理流程：
 *  将这两个32位数据转换为字节数组
 *  如果第0个字节是0xAA，则表示高压请求激活，HVreq置为true
 *  如果第0个字节是0xCC，则表示高压请求取消，HVreq置为false
 *  注释提示：字节7是计数器，用于消息校验（但代码中未使用）
 */
void chargerClass::handle108(uint32_t data[2])  // HV request
{
    uint8_t* bytes = (uint8_t*)data; // 将两个32位数据强制转换成字节指针
    if (bytes[0] == 0xAA) HVreq = true;  // 高压请求激活
    if (bytes[0] == 0xCC) HVreq = false; // 高压请求取消
    // 字节7为计数器，用于消息验证（此处未处理）
}

/**
 * 每100毫秒发送一次CAN消息ID 0x109，用于发送充电器状态信息
 * 参数：
 *  ChRun - 充电器是否运行标志
 *  can - CAN硬件接口指针，用于发送消息
 *
 * 消息内容（8字节）：
 *  字节0：操作模式
 *  字节1-2：高压电压值，低字节+高字节
 *  字节3-4：高压电压设定值，低字节+高字节
 *  字节5-6：高压功率设定值，低字节+高字节
 *  字节7：状态标志+计数器（4位状态 + 4位计数器）
 *      当充电器运行且充电类型为交流(AC)时，字节7高4位为0xA(1010)，低4位为计数器counter_109
 *      当充电器未运行时，字节7高4位为0xC(1100)，低4位为计数器counter_109
 *
 * 计数器counter_109范围0~15，循环递增，用于消息标识
 */
void chargerClass::Send100msMessages(bool ChRun, CanHardware* can)
{
    uint8_t bytes[8];
    // 读取参数
    uint16_t HVvolts = Param::GetInt(Param::udc);       // 高压电压值
    uint16_t HVspnt = Param::GetInt(Param::Voltspnt);   // 高压电压设定值
    uint16_t HVpwr = Param::GetInt(Param::Pwrspnt);     // 高压功率设定值

    // 填充消息数据
    bytes[0] = Param::GetInt(Param::opmode);             // 操作模式
    bytes[1] = (HVvolts & 0xFF);                         // 高压电压低字节
    bytes[2] = ((HVvolts & 0xFF00) >> 8);                // 高压电压高字节
    bytes[3] = (HVspnt & 0xFF);                          // 设定电压低字节
    bytes[4] = ((HVspnt & 0xFF00) >> 8);                 // 设定电压高字节
    bytes[5] = (HVpwr & 0xFF);                           // 功率设定低字节
    bytes[6] = ((HVpwr & 0xFF00) >> 8);                  // 功率设定高字节

    // 根据充电状态设置字节7
    if ((ChRun) && (Param::GetInt(Param::chgtyp) == AC))
        bytes[7] = ((0xA << 4) | counter_109);  // 充电器运行，状态码0xA，低4位计数器
    if (!ChRun)
        bytes[7] = ((0xC << 4) | counter_109);  // 充电器停止，状态码0xC，低4位计数器

    // 计数器递增，超过15重置为0
    counter_109++;
    if (counter_109 >= 0xF) counter_109 = 0;

    // 发送CAN消息，ID为0x109，数据长度8字节
    can->Send(0x109, (uint32_t*)bytes, 8);
}



#if 0
// 以下代码块是注释掉的测试或调试代码，包含CAN消息解析及发送的示例

/////////////////////////////////////////////////////////////////////////////////////////////////////
// 解析CAN消息示例说明：

// HV电流：前13位，除以20转换为安培单位
// HV电压：接下来的10位，除以2转换为伏特单位
// LV电流：接下来的8位，除以5转换为安培单位
// LV电压：接下来的8位，除以10转换为伏特单位

// 0x30A消息似乎与交流输入有关：
// AC电流：前12位，除以5（猜测）
// AC电压：接下来的8位，乘以2转换

////////////////////////////////////////////////////////////////////////////////////////////////////

void CheckCAN()
{
    if(Can1.available())
    {
        Can1.read(inFrame);
        if(inFrame.id == 0x212)
        {
            // 解析高压电流，位移和缩放处理
            uint16_t HVcur_temp = (uint16_t)(inFrame.data.bytes[0]<<8 | inFrame.data.bytes[1]);
            HVcur = (float)(HVcur_temp >> 3) * 0.05;

            // 解析高压电压
            uint16_t HVvol_temp = (uint16_t)((((inFrame.data.bytes[1]<<8 | inFrame.data.bytes[2])) >> 1) & 0x3ff);
            HVvol = (float)(HVvol_temp) * 0.5;

            // 解析低压电流
            uint16_t LVcur_temp = (uint16_t)(((inFrame.data.bytes[2]<<8 | inFrame.data.bytes[3]) >> 1) & 0x00ff);
            LVcur = (float)(LVcur_temp) * 0.2;

            // 解析低压电压
            uint16_t LVvol_temp = (uint16_t)(((inFrame.data.bytes[3]<<8 | inFrame.data.bytes[4]) >> 1) & 0x00ff);
            LVvol = (float)(LVvol_temp) * 0.1;
        }

        if(inFrame.id == 0x30A)
        {
            // 解析交流电流
            uint16_t ACcur_temp = (uint16_t)((inFrame.data.bytes[0]<<8 | inFrame.data.bytes[1]) >> 4);
            ACcur = (float)(ACcur_temp) * 0.2;

            // 解析交流电压
            uint16_t ACvol_temp = (uint16_t)(((inFrame.data.bytes[1]<<8 | inFrame.data.bytes[2]) >> 4) & 0x00ff);
            ACvol = (float)(ACvol_temp) * 2;
        }
    }
}

void Frames30MS()
{
    if(timer_Frames30.check())
    {
        // 30ms定时发送的CAN帧示例（部分注释掉的代码）

        // Coda充电器相关帧示例（已注释）

#if 0
        outFrame.id = 0x050;
        outFrame.length = 8;
        outFrame.extended = 0;
        outFrame.rtr = 1;
        outFrame.data.bytes[0] = 0x00;
        outFrame.data.bytes[1] = 0xDC;
        outFrame.data.bytes[2] = 0x0A;
        outFrame.data.bytes[3] = 0xF0;
        outFrame.data.bytes[4] = 0x00;
        outFrame.data.bytes[5] = 0x00;
        outFrame.data.bytes[6] = 0x96;
        outFrame.data.bytes[7] = 0x01;
        Can1.sendFrame(outFrame);
#endif

        // Lear充电器（Ampera）发送帧示例（已注释）
#if 0
        outFrame.id = 0x30E;
        outFrame.length = 1;
        outFrame.extended = 0;
        outFrame.rtr = 1;
        outFrame.data.bytes[0] = CHGCON;
        Can1.sendFrame(outFrame);
#endif
    }
}

void Frames200MS()
{
    if(timer_Frames200.check())
    {
        // 200ms定时发送的CAN帧示例，控制Lear充电器

        outFrame.id = 0x304;
        outFrame.length = 4;
        outFrame.extended = 0;
        outFrame.rtr = 1;
        outFrame.data.bytes[0] = 0x40;                     // 固定命令字节
        outFrame.data.bytes[1] = parameters.Cur * 20;     // 电流命令值，转换为单位后乘20
        Vol_temp = parameters.Vol * 2;
        outFrame.data.bytes[2] = highByte(Vol_temp);      // 电压高字节，电压乘以2
        outFrame.data.bytes[3] = lowByte(Vol_temp);       // 电压低字节

        Can1.sendFrame(outFrame);

        // 注释说明：
        // 数据字节2和3组合为0x0320，十进制800，除以2即为400V直流电压
    }
}
#endif
