#pragma once
#include <stdint.h>
#include "../config/BoardPins.h"

// ════════════════════════════════════════════════════════════════
// 按键 → CAN 报文映射表
// 以后要改按键数量、CAN ID、帧类型、数据内容，只改本文件即可，
// 不需要动 CanBus / KeyPad / 业务逻辑。
//
// 协议出处：机车 CAN 公版协议（总线 500 kbps，终端电阻 120Ω）
//   Msg Name  : 0x24      标准帧(11bit)，DLC = 8，Cycle Time = 100ms
//   方向      : R = ADAS 接收（本板发送，MBox/记录仪接收）
//   目标信号  : 「流媒体显示切换(目前仅 Mbox 支持)」
//               Start Byte B3 / Start Bit B3.0 / 8bit / End B3.7
//               Byte Order = Motorola LSB，unSigned  → 字节对齐，直接写 data[3]
//   0x41 后向为主背景,前左右侧三小图    0x42 前向为主背景,后左右侧三小图
//   0x43 左侧为主背景,前后右三小图      0x44 右侧为主背景,前后左三小图
//   0x45 组合页面,四图平铺              0x0  无效，保持当前显示状态
//
// ★★★ 重要：0x24 一条帧里还打包了其它控制信号 ★★★
//   B0  : 录像控制(B0.0-2) / 拍照控制(B0.3-5) / can总线控制(B0.6，1=总线静默!)
//   B1  : BSD / LCW / RCW / FCW 功能开关
//   B2  : 主机复位(B2.0) / 上下电提醒(B2.1) / 主机重启(B2.2) / wifi(B2.3-4)
//         / 格式化录像卡(B2.5)
//   所以除 B3 以外必须全填 0x00（0x00 = 无效/执行当前状态），
//   否则按一下页面键可能顺手复位主机、重启、格式化录像卡或让总线静默。
//
// 5 个键 = 5 个页面，绝对寻址直接点选：按哪个到哪个，与 MBox 当前页面无关，
// 因此不会出现「翻页式按键丢帧后永久错位」的问题。
// ════════════════════════════════════════════════════════════════

enum class KeySendMode : uint8_t {
    PRESS_ONLY = 0,     // 按下连发 KEY_TX_REPEAT 帧，松开不发（默认，页面切换用）
    PRESS_RELEASE = 1,  // 按下发 data，松开发 releaseData
    HOLD_REPEAT = 2     // 按下先发一帧，按住期间每 repeat_ms 再发一帧
};

struct KeyAction {
    const char* name;             // 日志用名称
    uint32_t    id;               // CAN ID（标准帧 0x000~0x7FF，扩展帧最大 0x1FFFFFFF）
    bool        extended;         // false = 11bit 标准帧，true = 29bit 扩展帧
    uint8_t     dlc;              // 数据长度 0~8
    uint8_t     data[8];          // 按下时发送的数据
    KeySendMode mode;             // 发送语义
    uint16_t    repeat_ms;        // 仅 HOLD_REPEAT 用，0 按 100ms 处理
    uint8_t     releaseData[8];   // 仅 PRESS_RELEASE 用（松开时发送）
};

// ★★★ 按键报文表：一行一个按键，顺序对应 S1..S5 ★★★
// data[3] 就是 B3 = 流媒体显示切换；其余字节保持 0x00（无效/执行当前状态）
static const KeyAction kKeyActions[KEY_COUNT] = {
    //  名称                                   CAN ID 扩展帧 DLC  data[8]（data[3]=B3）                  模式                       周期  松开数据
    { "S1 后向为主+前左右三小图", 0x24, false, 8, { 0x00, 0x00, 0x00, 0x41, 0, 0, 0, 0 }, KeySendMode::PRESS_ONLY, 0, { 0 } },
    { "S2 前向为主+后左右三小图", 0x24, false, 8, { 0x00, 0x00, 0x00, 0x42, 0, 0, 0, 0 }, KeySendMode::PRESS_ONLY, 0, { 0 } },
    { "S3 左侧为主+前后右三小图", 0x24, false, 8, { 0x00, 0x00, 0x00, 0x43, 0, 0, 0, 0 }, KeySendMode::PRESS_ONLY, 0, { 0 } },
    { "S4 右侧为主+前后左三小图", 0x24, false, 8, { 0x00, 0x00, 0x00, 0x44, 0, 0, 0, 0 }, KeySendMode::PRESS_ONLY, 0, { 0 } },
    { "S5 组合页面+四图平铺",     0x24, false, 8, { 0x00, 0x00, 0x00, 0x45, 0, 0, 0, 0 }, KeySendMode::PRESS_ONLY, 0, { 0 } },
};

static_assert(sizeof(kKeyActions) / sizeof(kKeyActions[0]) == KEY_COUNT,
              "KeyCanMap.h 的报文表行数必须等于 BoardPins.h 中的 KEY_COUNT");
