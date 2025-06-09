/*
 * 该文件是 Zombieverter 项目的一部分。
 *
 * 版权所有 (C) 2023 Damien Maguire
 *
 * 本程序是自由软件：你可以根据 GNU 通用公共许可证的条款重新发布和/或修改，
 * 许可证版本为第3版，或（由你选择）任何更高版本。
 *
 * 本程序的发布希望对你有用，但不提供任何保证，包括对适销性或适用性的隐含保证。
 *
 * 你应该已经收到 GNU 通用公共许可证的副本。如果没有，请访问 <http://www.gnu.org/licenses/>。
 */

#include "JLR_G1.h"   // 引入JLR_G1类的头文件

// 换挡杆各档位定义
#define JLR_Park 0       // P档（停车档）
#define JLR_Reverse 1    // R档（倒车档）
#define JLR_Neutral 2    // N档（空档）
#define JLR_Drive 3      // D档（驾驶档）
#define JLR_Sport 7      // S档（运动档）

// 车门锁状态定义
#define Unlocked 0x40    // 解锁状态
#define Locked 0x00      // 锁定状态

// 全局变量声明
uint8_t DirJLRG1 = 0;     // 当前换挡杆方向状态
uint8_t Cnt20ms = 0;      // 20毫秒计数器，用于定时发送CAN消息
uint8_t ShtdwnCnt = 0;    // 关机计数器，用于延迟关机处理

bool routine = 0;         // 状态标志变量，暂未使用

uint8_t Cnt3f3 = 0;       // 用于发送CAN报文的计数变量
uint8_t Cnt312;           // 存储接收到的CAN消息中的计数值

// 预定义的字节数组，用于组成不同档位的CAN报文数据
uint8_t byte4[16] = {0, 0x10, 0x20, 0x30, 0x40, 0x50, 0x60, 0x70, 0x80, 0x90, 0xA0, 0xB0, 0xC0, 0xD0, 0xE0, 0xF0};
uint8_t byte5P[16] = {0x21, 0xBC, 0x06, 0x9B, 0x6F, 0xF2, 0x48, 0xD5, 0xBD, 0x20, 0x9A, 0x07, 0x0F3, 0x6E, 0xD4, 0x49};
uint8_t byte5R[16] = {0x67, 0xFA, 0x40, 0xDD, 0x29, 0xB4, 0x0E, 0x93, 0xFB, 0x66, 0xDC, 0x41, 0xB5, 0x28, 0x92, 0x0F};
uint8_t byte5N[16] = {0xFB, 0x76, 0xCC, 0x51, 0xA5, 0x38, 0x82, 0x1F, 0x77, 0xEA, 0x52, 0xCD, 0x39, 0xA4, 0x1E, 0x83};
uint8_t byte5D[16] = {0xE8, 0x75, 0xCF, 0x52, 0xA6, 0x3B, 0x81, 0x1C, 0x74, 0xE9, 0x53, 0xCE, 0x3A, 0xA7, 0x1D, 0x80};
uint8_t byte5S[16] = {0xE2, 0x7F, 0xC5, 0x58, 0xAC, 0x31, 0x8B, 0x16, 0x7E, 0xE3, 0x59, 0xC4, 0x30, 0xAD, 0x17, 0x8A};


// 设置CAN接口指针，并注册要监听的CAN报文ID（0x312）
void JLR_G1::SetCanInterface(CanHardware* c)
{
    can = c;
    can->RegisterUserMessage(0x312); // 注册JLR Gen 1换挡杆报文ID
}


// 解析接收到的CAN消息，更新档位信息和计数器
void JLR_G1::DecodeCAN(int id, uint32_t* data)
{
    uint8_t* bytes = (uint8_t*)data;  // 将数据转换为字节数组方便处理
    if (id == 0x312) // 只处理ID为0x312的消息
    {
        Cnt312 = bytes[7] & 0x0F;    // 从字节7低4位读取计数值
        DirJLRG1 = bytes[3] >> 4;    // 从字节3高4位读取换挡方向（档位）
    }
}


// 发送换挡信息的CAN报文，根据当前档位组装对应的数据
void JLR_G1::sendcan()
{
    uint8_t bytes[8];

    // 如果当前操作模式不是运行模式，强制设置档位为停车档
    if(Param::GetInt(Param::opmode) == !MOD_RUN) DirJLRG1 = JLR_Park;

    // 字节0根据是否为停车档设置不同值
    if (DirJLRG1 == JLR_Park)
    {
        bytes[0] = 0x5C;
    }
    else
    {
        bytes[0] = 0x7C;
    }

    bytes[2] = Cnt3f3; // 计数值赋给字节2，用于CAN帧计数

    // 以下根据当前档位设置各字节的值，并更新成员变量gear

    if (DirJLRG1 == JLR_Park)
    {
        bytes[1] = 0x66;
        bytes[3] = Cnt3f3 + 130;
        bytes[4] = 0xFF;
        bytes[5] = 0x7F;
        bytes[6] = 0x00;
        bytes[7] = 0x80;

        this->gear = PARK;  // 设置档位状态
    }

    if (DirJLRG1 == JLR_Reverse)
    {
        bytes[1] = 0x24;
        bytes[3] = Cnt3f3 + 3;
        bytes[4] = 0xFE;
        bytes[5] = 0xFF;
        bytes[6] = 0x01;
        bytes[7] = 0x00;

        this->gear = REVERSE;
    }

    if (DirJLRG1 == JLR_Neutral)
    {
        bytes[1] = 0x25;
        bytes[3] = Cnt3f3 + 4;
        bytes[4] = 0xFD;
        bytes[5] = 0xFF;
        bytes[6] = 0x02;
        bytes[7] = 0x00;

        this->gear = NEUTRAL;
    }

    if (DirJLRG1 == JLR_Drive)
    {
        bytes[1] = 0x24;
        bytes[3] = Cnt3f3 + 6;
        bytes[4] = 0xFB;
        bytes[5] = 0xFF;
        bytes[6] = 0x04;
        bytes[7] = 0x00;

        this->gear = DRIVE;
    }

    if (DirJLRG1 == JLR_Sport)
    {
        bytes[1] = 0x24;
        bytes[3] = Cnt3f3 + 10;
        bytes[4] = 0xF7;
        bytes[5] = 0xFF;
        bytes[6] = 0x08;
        bytes[7] = 0x00;

        // 该档位未设置gear变量，可能是特殊处理
    }

    // 通过CAN接口发送ID为0x3F3的8字节数据包
    can->Send(0x3F3, bytes, 8);
}


// 每10毫秒调用一次，主要用于周期性发送CAN数据及计数管理
void JLR_G1::Task10Ms()
{
    // 只在关机计数器小于20时执行（延迟关机处理）
    if(ShtdwnCnt < 20)
    {
        Cnt20ms++; // 20ms计数加1
        if (Cnt20ms == 2) // 每40ms执行一次发送
        {
            sendcan(); // 发送CAN消息
            Cnt3f3++;  // 发送计数器递增

            // 如果当前档位为停车档，调整计数器以跳过某些值
            if (DirJLRG1 == JLR_Park)
            {
                if (Cnt3f3 == 0x02)
                {
                    Cnt3f3 = 0x04;
                }
            }

            // 计数器满15（0xF）后归零，循环使用
            if (Cnt3f3 == 0xF)
            {
                Cnt3f3 = 0x00;
            }

            Cnt20ms = 0; // 重置20ms计数器
        }
    }
}


// 每100毫秒调用一次，主要用于状态检查和关机计数管理
void JLR_G1::Task100Ms()
{
    // 如果操作模式为关闭，则强制档位为中立档
    if(Param::GetInt(Param::opmode) == MOD_OFF) this->gear = NEUTRAL;

    // 如果操作模式不是运行模式，则增加关机计数器
    if(Param::GetInt(Param::opmode) == !MOD_RUN)
    {
        if(ShtdwnCnt < 20) ShtdwnCnt++; // 关机计数器递增，最多20
    }
    else
    {
        ShtdwnCnt = 0; // 运行模式时复位关机计数器
    }
}


// 获取当前档位状态，返回true表示成功
bool JLR_G1::GetGear(Shifter::Sgear& outGear)
{
    outGear = gear;    // 输出当前档位状态
    return true;       // 告知调用者获取成功
}
