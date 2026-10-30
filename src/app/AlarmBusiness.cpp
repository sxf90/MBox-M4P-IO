#include "AlarmBusiness.h"
#include "../buzzer/Buzzer.h"
#include "../can/CanBus.h"

AlarmBusiness alarmBusiness;

namespace {
// 把 "预警类型 → 下标" 统一到这里，避免到处 static_cast
inline uint8_t idx(WarningType type) { return (uint8_t)type; }
constexpr uint8_t kTypeMax = 4;   // WarningType::FCW
}  // namespace

// ─── begin ───────────────────────────────────────────────────

bool AlarmBusiness::begin() {
    buzzer.begin();

    canBus.onFrame([](const CanFrame& frame) { alarmBusiness.onFrame(frame); });

    Serial.printf("[BUZ] 预警监听已启动：ID=0x%03X（标准帧，%u 字节），链路超时 %lums\n",
                  (unsigned)ALARM_MSG_ID, (unsigned)ALARM_MSG_DLC,
                  (unsigned long)ALARM_LINK_TIMEOUT_MS);
#if ALARM_BIT_ORDER_MSB
    Serial.println("[BUZ] 位序模式：字节内自 MSB 起");
#else
    Serial.println("[BUZ] 位序模式：字节内自 LSB 起（默认）");
#endif
    return true;
}

// ─── 接收：0x15 解码 + 类型级聚合 ────────────────────────────

void AlarmBusiness::onFrame(const CanFrame& frame) {
    if (frame.id != ALARM_MSG_ID) return;
    if (frame.extended != (ALARM_MSG_EXTENDED != 0)) return;
    if (frame.rtr) return;

    _lastRxMs = frame.timestamp_ms != 0 ? frame.timestamp_ms : millis();

    if (_linkLost) {
        _linkLost = false;
        Serial.println("[BUZ] 0x15 恢复上报，预警监听恢复正常");
    }

    if (frame.dlc < ALARM_MSG_DLC) {
        if (!_dlcWarned) {
            _dlcWarned = true;
            Serial.printf("[BUZ] 0x15 的 DLC=%u 不足 %u 字节，忽略该帧\n",
                          (unsigned)frame.dlc, (unsigned)ALARM_MSG_DLC);
        }
        return;
    }
    _rxFrames++;

    if (!_ready) {
        _ready = true;
        Serial.printf("[BUZ] 0x15 首帧到达（B2=0x%02X B3=0x%02X），预警监听就绪\n",
                      frame.data[2], frame.data[3]);
    }
#if ALARM_DEBUG_RAW
    Serial.printf("[BUZ] RAW 0x15: %02X %02X %02X %02X %02X %02X %02X %02X\n",
                  frame.data[0], frame.data[1], frame.data[2], frame.data[3],
                  frame.data[4], frame.data[5], frame.data[6], frame.data[7]);
#endif

    // 1) 变化前快照（含串口模拟叠加态）
    bool beforeActive[5];
    uint8_t beforeLevel[5];
    _snapshot(beforeActive, beforeLevel);

    // 2) 真实 0x15 到达 → 串口模拟态作废，以总线为准
    for (uint8_t t = 0; t <= kTypeMax; ++t) _simActive[t] = false;

    // 3) 逐信号更新
    for (uint8_t i = 0; i < ALARM_SIGNAL_COUNT; ++i) {
        const AlarmSignalDef& sig = ALARM_SIGNALS[i];
        const bool active = (alarmExtractField(frame.data, sig) == sig.triggerValue);
        if (_state[i] == (active ? 1 : 0)) continue;

        _state[i] = active ? 1 : 0;
        const uint8_t t = idx(sig.type);
        if (t <= kTypeMax) {
            if (active) {
                _activeCounts[t]++;
            } else if (_activeCounts[t] > 0) {
                _activeCounts[t]--;
            }
        }
        Serial.printf("[BUZ] 0x15 %s（%s L%u）%s\n", sig.name,
                      warningTypeName(sig.type), (unsigned)sig.level,
                      active ? "触发" : "消失");
    }

    // 4) 重算各类型最高等级
    for (uint8_t t = 0; t <= kTypeMax; ++t) {
        _levels[t] = _recalcLevel((WarningType)t);
    }

    // 5) 只有"类型整体"变化才通知蜂鸣器（避免 100ms 每帧重置滴滴滴节拍）
    _dispatch(beforeActive, beforeLevel);
}

// ─── 链路超时 ────────────────────────────────────────────────

void AlarmBusiness::tick() {
#if ALARM_LINK_TIMEOUT_MS > 0
    if (_lastRxMs == 0 || _linkLost) return;
    if ((uint32_t)(millis() - _lastRxMs) <= (uint32_t)ALARM_LINK_TIMEOUT_MS) return;

    _linkLost = true;
    Serial.printf("[BUZ] 超过 %lums 没有收到 0x15，判定预警链路中断，强制静音\n",
                  (unsigned long)ALARM_LINK_TIMEOUT_MS);
    _clearAll("链路超时");
#endif
}

// ─── 串口调试（不需要总线即可验证蜂鸣器）────────────────────

void AlarmBusiness::simulateFCW(bool active) {
    bool beforeActive[5];
    uint8_t beforeLevel[5];
    _snapshot(beforeActive, beforeLevel);

    _simActive[idx(WarningType::FCW)] = active;
    Serial.printf("[BUZ] 串口模拟 FCW %s\n", active ? "触发" : "消失");

    _dispatch(beforeActive, beforeLevel);
}

void AlarmBusiness::simulateRCW(bool active) {
    bool beforeActive[5];
    uint8_t beforeLevel[5];
    _snapshot(beforeActive, beforeLevel);

    _simActive[idx(WarningType::RCW)] = active;
    Serial.printf("[BUZ] 串口模拟 RCW %s\n", active ? "触发" : "消失");

    _dispatch(beforeActive, beforeLevel);
}

void AlarmBusiness::simulateClearAll() {
    _clearAll("串口模拟清除");
}

// ─── 状态打印 ────────────────────────────────────────────────

uint32_t AlarmBusiness::lastRxAgeMs() const {
    if (_lastRxMs == 0) return 0;
    return (uint32_t)(millis() - _lastRxMs);
}

void AlarmBusiness::printStatus() {
    CanBus::Stats st;
    canBus.getStats(st);

    Serial.printf("[BUZ] 蜂鸣器 GPIO%d：%s（输出 %s）| 模式=%s\n",
                  BUZZER_PIN, buzzer.isBeeping() ? "鸣响中" : "静音",
                  buzzer.outputOn() ? "ON" : "OFF", Buzzer::modeName(buzzer.mode()));

    if (_lastRxMs == 0) {
        Serial.printf("[BUZ] 预警状态：0x15 尚未收到任何帧（总线累计收帧 %lu）\n",
                      (unsigned long)st.rx);
        return;
    }

    bool active[5];
    uint8_t level[5];
    _snapshot(active, level);

    const bool fcwOn = active[idx(WarningType::FCW)];
    const bool rcwOn = active[idx(WarningType::RCW)];
    Serial.printf("[BUZ] 预警状态：FCW=%s%s RCW=%s%s BSD=%s LCW=%s\n",
                  fcwOn ? "ON" : "OFF", fcwOn ? (level[idx(WarningType::FCW)] >= 2 ? "(L2)" : "(L1)") : "",
                  rcwOn ? "ON" : "OFF", rcwOn ? (level[idx(WarningType::RCW)] >= 2 ? "(L2)" : "(L1)") : "",
                  active[idx(WarningType::BSD)] ? "ON" : "OFF",
                  active[idx(WarningType::LCW)] ? "ON" : "OFF");

    Serial.printf("[BUZ] 0x15 收帧 %lu，最近 %lums 前，链路 %s\n",
                  (unsigned long)_rxFrames, (unsigned long)lastRxAgeMs(),
                  _linkLost ? "中断(已强制静音)" : "正常");
}

// ─── 私有实现 ────────────────────────────────────────────────

void AlarmBusiness::_snapshot(bool activeOut[5], uint8_t levelOut[5]) const {
    for (uint8_t t = 0; t <= kTypeMax; ++t) {
        const bool busActive = (_activeCounts[t] > 0);
        activeOut[t] = busActive || _simActive[t];
        uint8_t lvl = _levels[t];
        if (lvl == 0 && _simActive[t]) lvl = 1;   // 串口模拟按一级预警处理
        levelOut[t] = lvl;
    }
}

uint8_t AlarmBusiness::_recalcLevel(WarningType type) const {
    uint8_t highest = 0;
    for (uint8_t i = 0; i < ALARM_SIGNAL_COUNT; ++i) {
        if (!_state[i]) continue;
        if (ALARM_SIGNALS[i].type != type) continue;
        if (ALARM_SIGNALS[i].level > highest) highest = ALARM_SIGNALS[i].level;
    }
    return highest;
}

void AlarmBusiness::_dispatch(const bool beforeActive[5], const uint8_t beforeLevel[5]) {
    bool afterActive[5];
    uint8_t afterLevel[5];
    _snapshot(afterActive, afterLevel);

    for (uint8_t t = 1; t <= kTypeMax; ++t) {
        const bool changed = (afterActive[t] != beforeActive[t]) ||
                             (afterActive[t] && afterLevel[t] != beforeLevel[t]);
        if (!changed) continue;

        WarningEvent evt;
        evt.type   = (WarningType)t;
        evt.active = afterActive[t];
        evt.level  = afterActive[t] ? afterLevel[t] : beforeLevel[t];
        if (evt.level == 0) evt.level = 1;
        buzzer.onWarning(evt);
    }
}

void AlarmBusiness::_clearAll(const char* reason) {
    bool beforeActive[5];
    uint8_t beforeLevel[5];
    _snapshot(beforeActive, beforeLevel);

    for (uint8_t i = 0; i < ALARM_SIGNAL_COUNT; ++i) _state[i] = 0;
    for (uint8_t t = 0; t <= kTypeMax; ++t) {
        _activeCounts[t] = 0;
        _levels[t] = 0;
        _simActive[t] = false;
    }

    // 高优先级先发"消失"事件，让蜂鸣器走正常下降沿（RCW 仍会补完当前这一拍）
    for (int8_t t = kTypeMax; t >= 1; --t) {
        if (!beforeActive[t]) continue;

        WarningEvent evt;
        evt.type   = (WarningType)t;
        evt.active = false;
        evt.level  = beforeLevel[t] != 0 ? beforeLevel[t] : 1;
        buzzer.onWarning(evt);
    }

    if (reason) Serial.printf("[BUZ] 预警全部清除（%s）\n", reason);
}
