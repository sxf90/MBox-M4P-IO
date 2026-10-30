#pragma once
#include <stdint.h>

// ════════════════════════════════════════════════════════════════
// 0x15「ADAS 状态广播」预警监听表 —— FCW / RCW → 蜂鸣器
//
// 出处：《机车 CAN 协议公版（对外释放客户）》表格中的 0x15 报文段。
//   方向 : S = ADAS 广播（MBox/记录仪 → 总线），本板只监听、不发送
//   周期 : 100ms      标准帧(11bit)      DLC = 8
//
// 本文件只描述「哪个字节的哪几位是哪种预警」，不关心怎么响。
// 响法（长鸣 / 滴滴滴 / 优先级 / 补一拍）在 buzzer/Buzzer 里实现，
// 与 VoiceBox 的 BuzzerManager 保持一致。
//
// 只有 B3.0~B3.1(RCW) 与 B3.2(FCW) 会驱动蜂鸣器（beep = true）；
// B2 的 BSD/LCW 四个字段也解码、打日志，但不发声（与 VoiceBox 一致）。
// ════════════════════════════════════════════════════════════════

#define ALARM_MSG_ID        0x15   // 预警状态帧 ID
#define ALARM_MSG_EXTENDED  0     // 0 = 标准帧
#define ALARM_MSG_DLC       8     // 至少 8 字节（用到 B3）

// 位序：0 = 字节内自 LSB 起（默认，与 0x24 表一致的线性编号）
//       1 = 字节内自 MSB 起（若现场实测确认 Bit 编号是 MSB 起，改成 1 即可）
#ifndef ALARM_BIT_ORDER_MSB
#define ALARM_BIT_ORDER_MSB 0
#endif

// ─── 预警类型（与 VoiceBox 的 WarningType 同名同值，便于对照）─────
enum class WarningType : uint8_t {
    NONE = 0,
    BSD  = 1,   // 盲区预警      —— 只监听，不发声
    LCW  = 2,   // 变道预警      —— 只监听，不发声
    RCW  = 3,   // 后车碰撞预警  —— 滴滴滴（150ms 响 / 150ms 停）
    FCW  = 4    // 前车碰撞预警  —— 长鸣（最高优先级）
};

// 一次预警状态变化（buzzer/Buzzer 只认这个结构）
struct WarningEvent {
    WarningType type   = WarningType::NONE;
    uint8_t     level  = 1;       // 1 = 一级预警, 2 = 二级预警
    bool        active = false;   // true = 触发(上升沿), false = 消失(下降沿)
};

// ─── 一行一个预警信号 ────────────────────────────────────────────
struct AlarmSignalDef {
    const char* name;          // 日志用名字
    WarningType type;          // 预警类型
    uint8_t     level;         // 该信号代表的预警等级
    uint8_t     byteIndex;     // 所在字节 0~7
    uint8_t     shift;         // 字段最低位在该字节内的位号（bit0 起）
    uint8_t     mask;           // 位宽掩码
    uint8_t     triggerValue;  // 等于该值即判为「触发」
    bool        beep;          // true = 驱动蜂鸣器
};

static const AlarmSignalDef ALARM_SIGNALS[] = {
    // ── B2：BSD / LCW（只监听、不发声）──────────────────────────
    { "BSD_L1_Left",  WarningType::BSD, 1, 2, 0, 0x03, 1, false },
    { "BSD_L2_Left",  WarningType::BSD, 2, 2, 0, 0x03, 2, false },
    { "BSD_L1_Right", WarningType::BSD, 1, 2, 2, 0x03, 1, false },
    { "BSD_L2_Right", WarningType::BSD, 2, 2, 2, 0x03, 2, false },
    { "LCW_L1_Left",  WarningType::LCW, 1, 2, 4, 0x03, 1, false },
    { "LCW_L2_Left",  WarningType::LCW, 2, 2, 4, 0x03, 2, false },
    { "LCW_L1_Right", WarningType::LCW, 1, 2, 6, 0x03, 1, false },
    { "LCW_L2_Right", WarningType::LCW, 2, 2, 6, 0x03, 2, false },

    // ── B3：RCW / FCW（驱动蜂鸣器）──────────────────────────────
    // B3.0~B3.1 后车碰撞预警 RCW：0x0 未触发 / 0x1 一级 / 0x2 二级
    { "RCW_L1",       WarningType::RCW, 1, 3, 0, 0x03, 1, true  },
    { "RCW_L2",       WarningType::RCW, 2, 3, 0, 0x03, 2, true  },
    // B3.2 前车碰撞预警 FCW：0x0 未触发 / 0x1 预警
    { "FCW",          WarningType::FCW, 1, 3, 2, 0x01, 1, true  },
};

constexpr uint8_t ALARM_SIGNAL_COUNT =
    (uint8_t)(sizeof(ALARM_SIGNALS) / sizeof(ALARM_SIGNALS[0]));

// B3.3~B3.7 还有 FDW / 起步提醒 LDVN / 车道偏航 / 头盔 / 预留，
// 当前业务不需要（不发声），需要时按上面格式加一行即可。

// 位宽：从掩码推出来（0x01 → 1 位，0x03 → 2 位 ……）
inline uint8_t alarmBitWidth(uint8_t mask) {
    uint8_t width = 0;
    while (mask) {
        width = (uint8_t)(width + (mask & 0x01));
        mask >>= 1;
    }
    return width;
}

// 从一帧数据里取出某个信号的值（0x15 的字段都落在单个字节内）
inline uint8_t alarmExtractField(const uint8_t* data, const AlarmSignalDef& sig) {
#if ALARM_BIT_ORDER_MSB
    const uint8_t width = alarmBitWidth(sig.mask);
    const uint8_t shift = (uint8_t)(8 - (sig.shift + width));
#else
    const uint8_t shift = sig.shift;
#endif
    return (uint8_t)((data[sig.byteIndex] >> shift) & sig.mask);
}

// 预警类型名称（日志用）
inline const char* warningTypeName(WarningType type) {
    switch (type) {
        case WarningType::BSD: return "BSD";
        case WarningType::LCW: return "LCW";
        case WarningType::RCW: return "RCW";
        case WarningType::FCW: return "FCW";
        default:               return "NONE";
    }
}
