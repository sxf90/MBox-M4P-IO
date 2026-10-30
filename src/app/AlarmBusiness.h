#pragma once
#include <Arduino.h>
#include "../can/CanFrame.h"
#include "../config/AlarmMap.h"

// ════════════════════════════════════════════════════════════════
// 业务层：监听 0x15 预警状态帧 → 解码 FCW/RCW → 驱动蜂鸣器
//
// 与按键业务（app/KeyCanBusiness）完全独立：
//   - 不引用 KeyPad / KeyCanMap / KeyCanBusiness 的任何东西
//   - 只注册 canBus.onFrame 回调，只写蜂鸣器引脚
//   - 按键发 0x24 的路径一行都不改
//
// 关键点：0x15 是 100ms 周期帧，绝不能"每帧都调 buzzer.onWarning"，
// 否则正在响的那一拍会被不停重置。这里做「类型级聚合」：
//   每个信号记活跃标志 → 每种预警类型统计活跃信号数 + 最高等级
//   → 只有"类型整体活跃态"或"最高等级"变化时才发一次 WarningEvent。
// （与 VoiceBox 的 SignalProcessor 行为一致）
// ════════════════════════════════════════════════════════════════
class AlarmBusiness {
public:
    // 初始化蜂鸣器 + 注册 canBus.onFrame 回调
    bool begin();

    // canBus 接收回调（在 canBus.poll() 中执行，必须短小）
    void onFrame(const CanFrame& frame);

    // loop() 中调用：0x15 链路超时检查
    void tick();

    // ─── 串口调试（不需要总线即可验证蜂鸣器）──────────────────
    void simulateFCW(bool active = true);
    void simulateRCW(bool active = true);
    void simulateClearAll();

    void printStatus();

    // 统计
    uint32_t rxFrames() const { return _rxFrames; }
    uint32_t lastRxAgeMs() const;
    bool fcwActive() const { return _activeCounts[(uint8_t)WarningType::FCW] > 0 || _simActive[(uint8_t)WarningType::FCW]; }
    bool rcwActive() const { return _activeCounts[(uint8_t)WarningType::RCW] > 0 || _simActive[(uint8_t)WarningType::RCW]; }
    uint8_t highestLevel(WarningType type) const { return _levels[(uint8_t)type]; }

private:
    // 当前"有效活跃态 / 等级"（总线聚合 + 串口模拟的叠加）
    void _snapshot(bool activeOut[5], uint8_t levelOut[5]) const;

    // 比对前后状态，只在变化时向蜂鸣器发事件
    void _dispatch(const bool beforeActive[5], const uint8_t beforeLevel[5]);

    // 某类型的最高活跃等级（扫 _state 得到）
    uint8_t _recalcLevel(WarningType type) const;

    // 全部清除（链路超时 / 串口模拟消失）
    void _clearAll(const char* reason);

    uint8_t  _state[ALARM_SIGNAL_COUNT] = {};  // 每个信号当前是否触发
    uint16_t _activeCounts[5] = {};            // 每种预警类型当前活跃信号数
    uint8_t  _levels[5]       = {};            // 每种预警类型当前最高等级
    bool     _simActive[5]    = {};            // 串口模拟的活跃态（下一帧真实 0x15 到达即作废）

    uint32_t _rxFrames = 0;
    uint32_t _lastRxMs = 0;
    bool     _ready    = false;   // 是否已收到首帧
    bool     _linkLost = false;   // 是否处于链路超时状态
    bool     _dlcWarned = false;
};

extern AlarmBusiness alarmBusiness;
