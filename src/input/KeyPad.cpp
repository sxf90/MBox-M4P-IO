#include "KeyPad.h"

namespace {
constexpr uint16_t SCAN_INTERVAL_MS = 5;   // 扫描节拍
}

const char* KeyPad::kindName(KeyEventKind kind) {
    switch (kind) {
        case KeyEventKind::PRESS:   return "按下";
        case KeyEventKind::RELEASE: return "松开";
        case KeyEventKind::HOLD:    return "长按";
        default:                    return "?";
    }
}

// ─── begin ───────────────────────────────────────────────────

bool KeyPad::begin(const uint8_t* pins, uint8_t count, bool comToGnd) {
    if (pins == nullptr || count == 0) return false;
    if (count > MAX_KEYS) count = MAX_KEYS;

    _count = count;
    _activeLow = comToGnd;
    _lastScanMs = millis();

    for (uint8_t i = 0; i < _count; ++i) {
        _pins[i] = pins[i];
        pinMode(_pins[i], _activeLow ? INPUT_PULLUP : INPUT_PULLDOWN);

        const bool pressed = (_activeLow ? digitalRead(_pins[i]) == LOW
                                         : digitalRead(_pins[i]) == HIGH);
        _raw[i] = pressed;
        // 上电时已按住的键视为初始状态，不补发"按下"事件
        _stable[i] = pressed;
        _changeMs[i] = _lastScanMs;
        _pressMs[i] = _lastScanMs;
    }

    _scanSnapshotReady = false;
    _ready = true;

    Serial.printf("[KEY] 初始化 %u 个按键（%s），消抖 %u ms\n",
                  (unsigned)_count,
                  _activeLow ? "内部上拉/COM 接 GND，按下=LOW"
                             : "内部下拉/COM 接 3V3，按下=HIGH",
                  (unsigned)KEY_DEBOUNCE_MS);
    for (uint8_t i = 0; i < _count; ++i) {
        Serial.printf("[KEY]   S%u -> GPIO%u\n", (unsigned)(i + 1), (unsigned)_pins[i]);
    }
    return true;
}

// ─── poll ────────────────────────────────────────────────────

void KeyPad::poll() {
    if (!_ready) return;

    const uint32_t now = millis();
    if (now - _lastScanMs < SCAN_INTERVAL_MS) return;
    _lastScanMs = now;

    for (uint8_t i = 0; i < _count; ++i) {
        const bool pressed = (_activeLow ? digitalRead(_pins[i]) == LOW
                                         : digitalRead(_pins[i]) == HIGH);

        // 原始电平变化 → 重新计时
        if (pressed != _raw[i]) {
            _raw[i] = pressed;
            _changeMs[i] = now;
        }

        // 未稳定够消抖时间 → 继续等
        if (now - _changeMs[i] < KEY_DEBOUNCE_MS) continue;
        if (_raw[i] == _stable[i]) continue;

        // 电平已稳定，确认状态切换
        _stable[i] = _raw[i];
        if (_stable[i]) {
            _pressMs[i] = now;
            _fire(i, KeyEventKind::PRESS, 0);
        } else {
            _fire(i, KeyEventKind::RELEASE, now - _pressMs[i]);
        }
    }
}

// ─── 查询 ────────────────────────────────────────────────────

bool KeyPad::isPressed(uint8_t index) const {
    if (!_ready || index >= _count) return false;
    return _stable[index];
}

uint32_t KeyPad::heldMs(uint8_t index) const {
    if (!_ready || index >= _count || !_stable[index]) return 0;
    return millis() - _pressMs[index];
}

// ─── 引脚探测 ────────────────────────────────────────────────

void KeyPad::debugScan() {
    if (!_ready) return;

    for (uint8_t i = 0; i < _count; ++i) {
        const bool high = digitalRead(_pins[i]) == HIGH;
        if (!_scanSnapshotReady || _scanSnapshot[i] != high) {
            _scanSnapshot[i] = high;
            Serial.printf("[SCAN] S%u -> GPIO%u = %s\n",
                          (unsigned)(i + 1), (unsigned)_pins[i],
                          high ? "HIGH(松开)" : "LOW(按下)");
        }
    }
    _scanSnapshotReady = true;
}

// ─── 事件派发 ────────────────────────────────────────────────

void KeyPad::_fire(uint8_t index, KeyEventKind kind, uint32_t heldMs) {
    if (!_handler) return;
    // C++11 下 KeyEvent 带默认成员初值、不是聚合类型，不能用花括号列表初始化
    KeyEvent evt;
    evt.index = index;
    evt.kind = kind;
    evt.heldMs = heldMs;
    _handler(evt);
}
