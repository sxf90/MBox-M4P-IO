#include "Buzzer.h"

Buzzer buzzer;

// ─── begin / stop ────────────────────────────────────────────

bool Buzzer::begin() {
    pinMode(BUZZER_PIN, OUTPUT);
    _writeOutput(false);            // 上电先静音，避免"啪"一声

    _mode       = Mode::NONE;
    _pendingEnd = false;
    _fcwActive  = false;
    _rcwActive  = false;
    _rcwOn      = false;
    _rcwToggleAt = 0;

    Serial.printf("[BUZ] 蜂鸣器 GPIO%d 初始化完成（%s触发，RCW %lums 响 / %lums 停）\n",
                  BUZZER_PIN, BUZZER_ACTIVE_HIGH ? "高电平" : "低电平",
                  (unsigned long)BUZZER_RCW_ON_MS, (unsigned long)BUZZER_RCW_OFF_MS);
    return true;
}

void Buzzer::stop() {
    _writeOutput(false);
    _mode        = Mode::NONE;
    _pendingEnd  = false;
    _fcwActive   = false;
    _rcwActive   = false;
    _rcwOn       = false;
    _rcwToggleAt = 0;
}

// ─── onWarning（逻辑与 VoiceBox BuzzerManager::onWarning 逐行一致）──

void Buzzer::onWarning(const WarningEvent& evt) {
    // BSD / LCW / NONE 不触发蜂鸣
    if (evt.type == WarningType::BSD || evt.type == WarningType::LCW ||
        evt.type == WarningType::NONE) {
        return;
    }

    // 更新活跃状态（仅 FCW / RCW）
    switch (evt.type) {
        case WarningType::FCW: _fcwActive = evt.active; break;
        case WarningType::RCW: _rcwActive = evt.active; break;
        default: break;
    }

    if (evt.active) {
        // 上升沿：检查新模式
        const Mode newMode = _currentMode();
        if (newMode == Mode::NONE) return;

        // 与当前模式相同 → 不打断正在响的那一拍，只取消"补一次后停止"
        if (_mode != Mode::NONE && _mode == newMode) {
            _pendingEnd = false;
            return;
        }

        _pendingEnd = false;
        _mode = newMode;

        if (_mode == Mode::FCW_LONG) {
            _writeOutput(true);
            Serial.println("[BUZ] FCW 长鸣开始");
        } else if (_mode == Mode::RCW_DIDIDI) {
            _rcwOn       = true;
            _writeOutput(true);
            _rcwToggleAt = millis() + BUZZER_RCW_ON_MS;
            Serial.println("[BUZ] RCW 滴滴滴开始");
        }
    } else {
        // 下降沿：检查是否还有其它活跃预警
        const Mode newMode = _currentMode();

        if (newMode != Mode::NONE && newMode != _mode) {
            // 还有更低优先级的预警 → 直接切过去
            _pendingEnd = false;
            _mode = newMode;

            if (_mode == Mode::FCW_LONG) {
                _writeOutput(true);
                Serial.println("[BUZ] 切换至 FCW 长鸣");
            } else if (_mode == Mode::RCW_DIDIDI) {
                _rcwOn       = true;
                _writeOutput(true);
                _rcwToggleAt = millis() + BUZZER_RCW_ON_MS;
                Serial.println("[BUZ] 切换至 RCW 滴滴滴");
            }
        } else if (newMode == Mode::NONE) {
            // 所有预警消失：补完当前这一拍再停
            _pendingEnd = true;
            Serial.println("[BUZ] 预警消失，补一拍后停止");
        }
    }
}

// ─── poll（推进 RCW 滴滴滴节拍 / 处理补完停止）──────────────────

void Buzzer::poll() {
    if (_mode == Mode::NONE) return;

    if (_mode == Mode::FCW_LONG) {
        // FCW 长鸣只是保持有效电平，不需要额外操作
        // 若 _pendingEnd 为 true（FCW 消失且再无其它预警）→ 立即停，不补拍
        if (_pendingEnd) {
            _writeOutput(false);
            _mode       = Mode::NONE;
            _pendingEnd = false;
            _rcwOn      = false;
            Serial.println("[BUZ] FCW 补完停止");
        }
        return;
    }

    if (_mode == Mode::RCW_DIDIDI) {
        const uint32_t now = millis();

        if (_pendingEnd) {
            // 下降沿后：补完当前这一 "滴" → 停止
            if (_rcwOn && now >= _rcwToggleAt) {
                _writeOutput(false);
                _mode       = Mode::NONE;
                _pendingEnd = false;
                _rcwOn      = false;
                Serial.println("[BUZ] RCW 补完停止");
                return;
            }
            // 当前处于"停"的阶段（两个滴之间）→ 立即停
            if (!_rcwOn) {
                _writeOutput(false);
                _mode       = Mode::NONE;
                _pendingEnd = false;
                _rcwOn      = false;
                Serial.println("[BUZ] RCW 补完停止(OFF期间)");
                return;
            }
        }

        // 正常叮滴节拍翻转
        if (now >= _rcwToggleAt) {
            if (_rcwOn) {
                _writeOutput(false);
                _rcwOn       = false;
                _rcwToggleAt = now + BUZZER_RCW_OFF_MS;
            } else {
                _writeOutput(true);
                _rcwOn       = true;
                _rcwToggleAt = now + BUZZER_RCW_ON_MS;
            }
        }
    }
}

// ─── 私有方法 ────────────────────────────────────────────────

Buzzer::Mode Buzzer::_currentMode() const {
    if (_fcwActive) return Mode::FCW_LONG;      // FCW 最高优先级
    if (_rcwActive) return Mode::RCW_DIDIDI;
    return Mode::NONE;
}

void Buzzer::_writeOutput(bool on) {
    const bool high = BUZZER_ACTIVE_HIGH ? on : !on;
    digitalWrite(BUZZER_PIN, high ? HIGH : LOW);
    _output = on;
}

const char* Buzzer::modeName(Mode mode) {
    switch (mode) {
        case Mode::FCW_LONG:   return "FCW 长鸣";
        case Mode::RCW_DIDIDI: return "RCW 滴滴滴";
        case Mode::NONE:
        default:               return "静音";
    }
}
