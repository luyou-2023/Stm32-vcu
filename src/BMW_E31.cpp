/*
 * 该文件属于 Zombieverter 项目。
 *
 * 版权所有 (C) 2023 Damien Maguire
 *
 * 本程序是自由软件：您可以根据 GNU 通用公共许可证第 3 版或其任何后续版本，
 * 重新分发和/或修改本程序。
 *
 * 本程序按“希望有用”的原则发布，但不提供任何形式的担保，
 * 包括适销性或适合特定用途的隐含担保。
 *
 * 您应当已收到 GNU 通用公共许可证副本。
 * 若没有，请访问 <http://www.gnu.org/licenses/>。
 */

#include "BMW_E31.h"         // 包含 BMW_E31 类的声明
#include "hwinit.h"          // 硬件初始化相关
#include <libopencm3/stm32/timer.h> // STM32 定时器相关函数
#include <libopencm3/stm32/gpio.h>  // STM32 GPIO 相关函数

/*
 * E31 840CI 仪表转速表参考频率：
 * 1000 转/分 = 70 Hz
 * 2000 转/分 = 140 Hz
 * 5000 转/分 = 345 Hz
 * 6000 转/分 = 413 Hz
 */

// 作为初始化函数使用
void BMW_E31::SetCanInterface(CanHardware* c)
{
    can = c;                   // 设置 CAN 硬件接口指针
    tim_setup();               // 初始化定时器1
    timer_disable_counter(TIM1); // 禁用定时器，待用时再启动
    // 这里尝试复用 Lexus GS450h 油泵的 PWM 输出信号来驱动转速表
    // 注意：这会导致 E31 与 GS450H 不能同时使用此功能
    timerIsRunning = false;    // 定时器状态标记，初始未启动

    can->RegisterUserMessage(0x153); // 注册 CAN ID 0x153 的消息（ASC消息），待确认具体含义
}

// 设置转速表的转速值，单位是转/分（RPM）
void BMW_E31::SetRevCounter(int speed)
{
    uint16_t speed_input = speed;
    speed_input = MAX(750, speed_input);   // 限制最小转速为 750 RPM
    speed_input = MIN(7500, speed_input);  // 限制最大转速为 7500 RPM
    // 计算定时器周期，TODO: 需要找到准确系数或改为参数化
    // 当前系数导致 750 RPM 时大约 52Hz
    timerPeriod = 30000000 / speed_input;
    timer_set_period(TIM1, timerPeriod);          // 设置定时器周期
    timer_set_oc_value(TIM1, TIM_OC1, timerPeriod / 2); // 设置输出比较值，50% 占空比
}

// 设置温度表的显示（当前未实现具体逻辑）
void BMW_E31::SetTemperatureGauge(float temp)
{
    float dc = temp * 10; // TODO: 需找到合适的比例系数，期望输出0~0.5区间
    // 未来希望使用数字电位器控制
    dc = dc; // 防止编译器警告（目前无实际功能）
}

// 解码 CAN 消息
void BMW_E31::DecodeCAN(int id, uint32_t* data)
{
    uint8_t* bytes = (uint8_t*)data; // 将数据转换为字节数组，便于解析

    if (id == 0x153) // 如果是 0x153 的 ASC1 消息
    {
        // 车速信号，单位 km/h
        // 计算方法: ((高字节 * 256) + 低字节 - 0x160) * 0.0625
        // 最小值 0x160 表示 0 km/h
        float road_speed = 0.0625f * (((bytes[2] << 8) | (bytes[1])) - 0x160);

        Param::SetFloat(Param::Veh_Speed, road_speed); // 保存车速参数
    }
}

// 发送 EGS 0x43B 消息，常用于自动变速器控制
void BMW_E31::EGSMsg43B()
{
    uint8_t bytes[3];

    bytes[0] = 0x46;  // 数据字节1
    bytes[1] = 0x00;  // 数据字节2
    bytes[2] = 0x00;  // 数据字节3

    can->Send(0x43B, (uint32_t*)bytes, 3); // 发送 CAN 消息 ID 0x43B，3字节数据
}

// 发送变速箱状态消息（CAN ID 0x43F）
// gear参数代表档位，-1 反向，0 空挡，1 驱动，默认其它
void BMW_E31::EGSMsg43F(int8_t gear)
{
    uint8_t bytes[8];

    // 参考来源：BimmerForums 讨论帖
    bytes[0] = 0x81; // 固定值，不影响仪表

    // 根据档位设置字节1的值
    switch (gear)
    {
    case -1: // 倒档
        bytes[1] = 0x07;
        break;
    case 0:  // 空挡
        bytes[1] = 0x06;
        break;
    case 1:  // 驱动档（D）
        bytes[1] = 0x05;
        break;
    default: // 其它档位，显示为P档
        bytes[1] = 0x08;
        break;
    }

    bytes[2] = 0xFF; // 不显示任何特殊符号
    bytes[3] = 0xFF; // 不影响仪表
    bytes[4] = 0x00; // 不影响仪表
    bytes[5] = 0x80; // 清除档位警告灯
    bytes[6] = 0xFF; // 不影响仪表
    bytes[7] = 0xFF; // 不影响仪表

    can->Send(0x43F, bytes, 8); // 发送变速器状态消息
}

// 1毫秒任务（当前未实现）
void BMW_E31::Task1Ms()
{

}

// 10毫秒任务，检测点火开关状态并发送相关消息
void BMW_E31::Task10Ms()
{
    if(DigIo::t15_digi.Get() == 1) // 点火开关接通
    {
        EGSMsg43B(); // 发送变速箱消息43B
        if (Param::GetBool(Param::Transmission)) // 如果变速箱使能
            EGSMsg43F(Param::GetInt(Param::dir)); // 发送当前档位消息43F
    }
}

// 100毫秒任务，控制定时器启动和关闭
void BMW_E31::Task100Ms()
{
    if(Param::GetInt(Param::T15Stat) && !timerIsRunning)
    {
        timer_enable_counter(TIM1); // 点火接通且定时器未启动，启动定时器
        timerIsRunning = true;
    }
    else if (!Param::GetInt(Param::T15Stat) && timerIsRunning)
    {
        timer_disable_counter(TIM1); // 点火断开且定时器正在运行，关闭定时器
        timerIsRunning = false;
    }
}

// 返回是否点火开关已接通
bool BMW_E31::Ready()
{
    return DigIo::t15_digi.Get();
}

// 返回是否发动机启动信号为真
bool BMW_E31::Start()
{
    return Param::GetBool(Param::din_start);
}
