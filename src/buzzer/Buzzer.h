#pragma once
#include <Arduino.h>
#include "../config/BoardPins.h"
#include "../config/AlarmMap.h"

// ════════════════════════════════════════════════════════════════
// 预警提示蜂鸣器（从 VoiceBox 的 BuzzerManager 移植，FCW/RCW 逻辑原样保留）
//
//   FCW 触发 → 连续长鸣（GPIO 持续有效电平）
//   RCW 触发 → 滴滴滴（响 BUZZER_RCW_ON_MS / 停 BUZZER_RCW_OFF_MS 交替）
//   优先级   → FCW > RCW
//   上升沿   → 开始蜂鸣（同模式不打断正在响的那一拍）
//   下降沿   → 补完当前这一拍后停止（在"停"的阶段则立即停）
//
// 与音频 / 三级预警灯语无关：本工程只有蜂鸣器一种输出。
// 输入的 WarningEvent 来自 app/AlarmBusiness（0x15 解码），本类不碰 CAN。
// ════════════════════════════════════════════════════════════════

#ifndef BUZZER_RCW_ON_MS
#define BUZZER_RCW_ON_MS   150   // 滴滴滴 - 响的时长(ms)
#endif
#ifndef BUZZER_RCW_OFF_MS
#define BUZZER_RCW_OFF_MS  150   // 滴滴滴 - 间隔时长(ms)
#endif

class Buzzer {
public:
    enum class Mode : uint8_t {
        NONE = 0,      // 空闲（不响）
        FCW_LONG = 1,  // FCW 长鸣
        RCW_DIDIDI = 2 // RCW 滴滴滴
    };

    // 初始化引脚为输出并静音
    bool begin();

    // 停止蜂鸣并回到空闲（清空 FCW/RCW 活跃状态）
    void stop();

    // 预警事件入口（由 app/AlarmBusiness 在状态变化时调用）
    void onWarning(const WarningEvent& evt);

    // loop() 中每轮调用：推进 RCW 滴滴滴节拍
    void poll();

    bool isBeeping() const { return _mode != Mode::NONE; }
    Mode mode() const { return _mode; }
    bool outputOn() const { return _output; }

    static const char* modeName(Mode mode);

private:
    // 根据当前活跃预警决定蜂鸣模式（FCW 优先）
    Mode _currentMode() const;

    // 实际控制 GPIO（按 BUZZER_ACTIVE_HIGH 换算电平）
    void _writeOutput(bool on);

    bool _fcwActive = false;
    bool _rcwActive = false;

    Mode     _mode       = Mode::NONE;
    bool     _pendingEnd = false;   // true = 预警已消失，等这一拍响完就停

    uint32_t _rcwToggleAt = 0;      // 下一次翻转（响↔停）的时间
    bool     _rcwOn       = false;  // 当前处于"响"还是"停"的阶段

    bool     _output = false;       // 当前输出状态（日志/统计用）
};

extern Buzzer buzzer;
