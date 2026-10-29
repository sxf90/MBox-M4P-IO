#pragma once
#include <Arduino.h>
#include <functional>

// 按键事件类型
enum class KeyEventKind : uint8_t {
    PRESS = 0,    // 按下（消抖确认后）
    RELEASE = 1,  // 松开
    HOLD = 2      // 保留：长按判定（当前默认不使用）
};

struct KeyEvent {
    uint8_t      index = 0;         // 按键序号 0~(count-1)，对应 S1~S8（本工程用 S1~S5）
    KeyEventKind kind = KeyEventKind::PRESS;
    uint32_t     heldMs = 0;        // 松开时携带按住时长
};

using KeyEventHandler = std::function<void(const KeyEvent&)>;

// 独立按键扫描：内部上拉/下拉 + 时间消抖 + 边沿事件
// 只负责"哪个键被按下了"，不关心按下之后做什么。
class KeyPad {
public:
    static constexpr uint8_t MAX_KEYS = 8;

    // comToGnd = true  -> INPUT_PULLUP，按下为 LOW（默认）
    // comToGnd = false -> INPUT_PULLDOWN，按下为 HIGH
    bool begin(const uint8_t* pins, uint8_t count, bool comToGnd = true);

    // loop() 中调用，内部按 5ms 节拍扫描（不使用 delay）
    void poll();

    void onEvent(KeyEventHandler handler) { _handler = handler; }

    bool isPressed(uint8_t index) const;
    uint32_t heldMs(uint8_t index) const;
    uint8_t count() const { return _count; }

    // 引脚探测模式：打印每个按键对应的 GPIO 电平变化（核对接线用）
    void debugScan();

    static const char* kindName(KeyEventKind kind);

private:
    void _fire(uint8_t index, KeyEventKind kind, uint32_t heldMs);

    uint8_t          _pins[MAX_KEYS] = {};
    uint8_t          _count = 0;
    bool             _activeLow = true;
    bool             _raw[MAX_KEYS] = {};        // 最近一次原始采样（true = 按下）
    bool             _stable[MAX_KEYS] = {};     // 消抖后的稳定状态
    uint32_t         _changeMs[MAX_KEYS] = {};   // 原始电平最近一次变化时间
    uint32_t         _pressMs[MAX_KEYS] = {};    // 稳定按下的时间
    bool             _scanSnapshot[MAX_KEYS] = {};
    bool             _scanSnapshotReady = false;
    uint32_t         _lastScanMs = 0;
    bool             _ready = false;
    KeyEventHandler  _handler;
};
