#include "KeyCanBusiness.h"
#include "../can/CanBus.h"

KeyCanBusiness keyCanBusiness;

namespace {

void printFrameHead(const KeyAction& action) {
    if (action.extended)
        Serial.printf("ID=0x%08lX 扩展帧", (unsigned long)action.id);
    else
        Serial.printf("ID=0x%03lX 标准帧", (unsigned long)action.id);
}

}  // namespace

// ─── begin ───────────────────────────────────────────────────

void KeyCanBusiness::begin(const KeyPad* pad) {
    _pad = pad;
    for (uint8_t i = 0; i < KeyPad::MAX_KEYS; ++i) {
        _nextRepeatMs[i]  = 0;
        _pendingLeft[i]   = 0;
        _pendingNextMs[i] = 0;
        _lastPressMs[i]   = 0;
    }
    _lastIndex = 0xFF;
    _lastKeepAliveMs = millis();
    _running = true;

    Serial.println("[KEY] 按键 → CAN 报文映射表：");
    printTable();
    Serial.printf("[KEY] 按下连发 %u 帧，帧间隔 %u ms，连按抑制 %u ms\n",
                  (unsigned)KEY_TX_REPEAT, (unsigned)KEY_TX_REPEAT_GAP_MS,
                  (unsigned)KEY_PAGE_MUTEX_MS);
#if CAN_CTRL_KEEPALIVE_MS > 0
    Serial.printf("[KEY] 0x24 保活帧：每 %u ms 一帧（B3=0x00，不改变 MBox 页面）\n",
                  (unsigned)CAN_CTRL_KEEPALIVE_MS);
#else
    Serial.println("[KEY] 0x24 保活帧：关闭（只在按键时发送，上电不动 MBox 页面）");
#endif
}

void KeyCanBusiness::printTable() const {
    for (uint8_t i = 0; i < KEY_COUNT; ++i) {
        const KeyAction& action = kKeyActions[i];
        const uint8_t dlc = action.dlc > 8 ? 8 : action.dlc;

        Serial.printf("[KEY]   S%u (GPIO%u) -> ", (unsigned)(i + 1),
                      (unsigned)KEY_PINS[i]);
        printFrameHead(action);
        Serial.printf(" DLC=%u [", (unsigned)dlc);
        for (uint8_t b = 0; b < dlc; ++b) {
            Serial.printf("%02X%s", action.data[b], (b + 1 < dlc) ? " " : "");
        }
        Serial.printf("]  %s", action.name);
        switch (action.mode) {
            case KeySendMode::PRESS_RELEASE: Serial.print("（按下/松开各一帧）"); break;
            case KeySendMode::HOLD_REPEAT:
                Serial.printf("（按住每 %u ms 一帧）",
                              (unsigned)(action.repeat_ms ? action.repeat_ms : 100));
                break;
            case KeySendMode::PRESS_ONLY:
            default:
                Serial.printf("（按下连发 %u 帧，间隔 %u ms）",
                              (unsigned)KEY_TX_REPEAT, (unsigned)KEY_TX_REPEAT_GAP_MS);
                break;
        }
        Serial.println();
    }
}

// ─── 按键事件 ────────────────────────────────────────────────

void KeyCanBusiness::onKeyEvent(const KeyEvent& evt) {
    if (!_running || evt.index >= KEY_COUNT) return;

    const KeyAction& action = kKeyActions[evt.index];

    if (evt.kind == KeyEventKind::PRESS) {
        const uint32_t now = millis();

        // 连按抑制：同一个键 KEY_PAGE_MUTEX_MS 内重复确认按下只算一次
        if (_lastPressMs[evt.index] != 0 &&
            now - _lastPressMs[evt.index] < KEY_PAGE_MUTEX_MS) {
            Serial.printf("[KEY] %s 连按抑制（%lu ms 内重复按下，忽略）\n", action.name,
                          (unsigned long)(now - _lastPressMs[evt.index]));
            return;
        }
        _lastPressMs[evt.index] = now;
        _press(evt.index);
        return;
    }

    if (evt.kind == KeyEventKind::RELEASE) {
        if (action.mode == KeySendMode::PRESS_RELEASE) _sendAction(evt.index, false);
        _nextRepeatMs[evt.index] = 0;
        // 注意：这里不清 _pendingLeft —— 快按时剩下没发完的补发帧仍要发完
        Serial.printf("[KEY] %s 松开（按住 %lu ms）\n", action.name,
                      (unsigned long)evt.heldMs);
    }
}

// ─── 按下处理（真实按键与串口模拟共用）───────────────────────

void KeyCanBusiness::_press(uint8_t index) {
    const KeyAction& action = kKeyActions[index];
    const uint32_t now = millis();

    _cancelPending(index);   // 页面互斥：作废同一条报文里其它键没发完的帧
    _lastIndex = index;

    _sendAction(index, true);   // 第 1 帧立即发送

    if (KEY_TX_REPEAT > 1) {
        _pendingLeft[index]   = (uint8_t)(KEY_TX_REPEAT - 1);
        _pendingNextMs[index] = now + KEY_TX_REPEAT_GAP_MS;
    } else {
        _pendingLeft[index] = 0;
    }

    if (action.mode == KeySendMode::HOLD_REPEAT) {
        const uint16_t period = action.repeat_ms ? action.repeat_ms : 100;
        _nextRepeatMs[index] = now + period;
    }
}

// ─── 页面互斥 ────────────────────────────────────────────────

void KeyCanBusiness::_cancelPending(uint8_t exceptIndex) {
    const KeyAction& action = kKeyActions[exceptIndex];
    for (uint8_t i = 0; i < KEY_COUNT; ++i) {
        if (i == exceptIndex || _pendingLeft[i] == 0) continue;
        // 只有同一条报文（同 ID + 同帧类型）才需要互斥，其它报文互不影响
        if (kKeyActions[i].id != action.id) continue;
        if (kKeyActions[i].extended != action.extended) continue;

        Serial.printf("[KEY] %s 的补发帧作废（已被 %s 覆盖）\n",
                      kKeyActions[i].name, action.name);
        _pendingLeft[i] = 0;
    }
}

// ─── 周期发送 ────────────────────────────────────────────────

void KeyCanBusiness::tick() {
    if (!_running) return;

    const uint32_t now = millis();
    bool busy = false;

    for (uint8_t i = 0; i < KEY_COUNT; ++i) {
        const KeyAction& action = kKeyActions[i];

        // 1) 按下后的补发帧（非阻塞推进，不用 delay）
        if (_pendingLeft[i] > 0) {
            busy = true;
            if (now >= _pendingNextMs[i]) {
                _sendAction(i, true);
                --_pendingLeft[i];
                _pendingNextMs[i] = now + KEY_TX_REPEAT_GAP_MS;
            }
        }

        // 2) HOLD_REPEAT 长按连发
        if (action.mode != KeySendMode::HOLD_REPEAT) continue;
        if (_pad == nullptr || !_pad->isPressed(i)) continue;

        busy = true;
        if (_nextRepeatMs[i] == 0) {
            _nextRepeatMs[i] = now + (action.repeat_ms ? action.repeat_ms : 100);
            continue;
        }
        if (now < _nextRepeatMs[i]) continue;

        _sendAction(i, true);
        _nextRepeatMs[i] = now + (action.repeat_ms ? action.repeat_ms : 100);
    }

#if CAN_CTRL_KEEPALIVE_MS > 0
    // 3) 空闲保活帧：B3 = 0x00，协议定义「保持当前显示状态」，不会改变 MBox 页面。
    //    补发帧没发完时不插队，避免 0x41 和 0x00 交替出现。
    if (!busy && now - _lastKeepAliveMs >= CAN_CTRL_KEEPALIVE_MS) {
        _lastKeepAliveMs = now;
        uint8_t idle[8] = { 0 };
        canBus.transmit(kKeyActions[0].id, idle, kKeyActions[0].dlc,
                        kKeyActions[0].extended);
    }
#else
    (void)busy;
#endif
}

// ─── 串口模拟按键 ────────────────────────────────────────────

void KeyCanBusiness::simulatePress(uint8_t index) {
    if (index >= KEY_COUNT) return;
    Serial.printf("[SIM] 模拟 %s 按下\n", kKeyActions[index].name);
    _press(index);   // 与真实按键走同一条路径（含补发帧）
}

// ─── 查询 ────────────────────────────────────────────────────

const char* KeyCanBusiness::lastPageName() const {
    if (_lastIndex >= KEY_COUNT) return "（尚未按过）";
    return kKeyActions[_lastIndex].name;
}

const KeyAction* KeyCanBusiness::lastAction() const {
    if (_lastIndex >= KEY_COUNT) return nullptr;
    return &kKeyActions[_lastIndex];
}

// ─── 实际发送 ────────────────────────────────────────────────

bool KeyCanBusiness::_sendAction(uint8_t index, bool pressed) {
    const KeyAction& action = kKeyActions[index];
    const uint8_t* data = pressed ? action.data : action.releaseData;
    const uint8_t dlc = action.dlc > 8 ? 8 : action.dlc;

    Serial.printf("[KEY] %s %s → ", action.name, pressed ? "按下" : "松开");
    printFrameHead(action);
    Serial.printf(" DLC=%u [", (unsigned)dlc);
    for (uint8_t i = 0; i < dlc; ++i) {
        Serial.printf("%02X%s", data[i], (i + 1 < dlc) ? " " : "");
    }
    Serial.print("] ");

    const bool ok = canBus.transmit(action.id, data, dlc, action.extended);
    if (ok) {
        Serial.println("发送成功");
    } else {
        CanBus::Stats stats;
        canBus.getStats(stats);
        Serial.printf("发送失败（已发 %lu 帧 / 失败 %lu 次 / TEC=%lu）\n",
                      (unsigned long)stats.tx, (unsigned long)stats.txFailed,
                      (unsigned long)stats.tec);
    }
    return ok;
}
