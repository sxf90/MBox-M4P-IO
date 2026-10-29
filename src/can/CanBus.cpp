#include "CanBus.h"

CanBus canBus;

namespace {

twai_mode_t toTwaiMode(CanBus::Mode mode) {
    switch (mode) {
        case CanBus::Mode::NO_ACK:      return TWAI_MODE_NO_ACK;
        case CanBus::Mode::LISTEN_ONLY: return TWAI_MODE_LISTEN_ONLY;
        case CanBus::Mode::NORMAL:
        default:                        return TWAI_MODE_NORMAL;
    }
}

}  // namespace

// ─── 模式名称（日志用）────────────────────────────────────────

const char* CanBus::modeName(Mode mode) {
    switch (mode) {
        case Mode::NORMAL:      return "NORMAL(正常收发)";
        case Mode::NO_ACK:      return "NO_ACK(单机自测)";
        case Mode::LISTEN_ONLY: return "LISTEN_ONLY(只听)";
        default:                return "UNKNOWN";
    }
}

// ─── begin ───────────────────────────────────────────────────

bool CanBus::begin() {
    // Config 的默认成员初值引用 config/BoardPins.h 的宏，
    // 不能在类定义内做默认实参，所以放在这里构造
    return begin(Config());
}

bool CanBus::begin(const Config& cfg) {
    if (_running) stop();
    _cfg = cfg;

    twai_general_config_t general = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)_cfg.tx, (gpio_num_t)_cfg.rx, toTwaiMode(_cfg.mode));
    // 默认队列只有 5 帧，按键连按 / 总线繁忙时必须放大
    general.tx_queue_len = _cfg.tx_queue_len;
    general.rx_queue_len = _cfg.rx_queue_len;

    twai_timing_config_t timing = _timingConfig(_cfg.baudrate);
    twai_filter_config_t filter = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&general, &timing, &filter) != ESP_OK) {
        Serial.println("[CAN] 驱动安装失败");
        return false;
    }
    if (twai_start() != ESP_OK) {
        Serial.println("[CAN] 启动失败");
        twai_driver_uninstall();
        return false;
    }

    _stats = Stats{};
    _lastWatchdogMs = millis();
    _running = true;

    Serial.printf("[CAN] 启动成功：%lu bps，%s，TX=GPIO%d RX=GPIO%d，队列 TX=%u RX=%u\n",
                  (unsigned long)_cfg.baudrate, modeName(_cfg.mode),
                  (int)_cfg.tx, (int)_cfg.rx,
                  (unsigned)_cfg.tx_queue_len, (unsigned)_cfg.rx_queue_len);
    return true;
}

// ─── stop ────────────────────────────────────────────────────

void CanBus::stop() {
    if (!_running) return;
    twai_stop();
    twai_driver_uninstall();
    _running = false;
    Serial.println("[CAN] 已停止");
}

// ─── poll ────────────────────────────────────────────────────

void CanBus::poll() {
    if (!_running) return;
    _drainRx();
    _watchdog();
}

// ─── 发送 ────────────────────────────────────────────────────

bool CanBus::transmit(uint32_t id, const uint8_t* data, uint8_t dlc,
                      bool extended, bool selfTest) {
    if (!_running) return false;

    if (_cfg.mode == Mode::LISTEN_ONLY) {
        Serial.println("[CAN] 只听模式下不能发送");
        _stats.txFailed++;
        return false;
    }
    if (dlc > 8) dlc = 8;
    if (dlc > 0 && data == nullptr) {
        _stats.txFailed++;
        return false;
    }
    if (!extended && id > 0x7FF) {
        Serial.printf("[CAN] 标准帧 ID 超范围: 0x%lX（需要扩展帧请置 extended=true）\n",
                      (unsigned long)id);
        _stats.txFailed++;
        return false;
    }

    twai_message_t msg = {};
    msg.identifier = id;
    msg.extd = extended ? 1 : 0;
    msg.rtr = 0;
    msg.ss = 0;                        // 非单次发送：允许控制器自动重试
    msg.self = selfTest ? 1 : 0;       // 自发自收（台架验证链路用）
    msg.data_length_code = dlc;
    if (dlc > 0) memcpy(msg.data, data, dlc);

    if (twai_transmit(&msg, 0) == ESP_OK) {
        _stats.tx++;
        return true;
    }

    // 队列满 / 无 ACK / 总线关闭
    _stats.txFailed++;
    return false;
}

// ─── 接收排空 ────────────────────────────────────────────────

void CanBus::_drainRx() {
    twai_message_t msg;
    for (uint8_t i = 0; i < _cfg.max_frames_per_poll; ++i) {
        if (twai_receive(&msg, 0) != ESP_OK) break;

        CanFrame frame;
        frame.id = msg.identifier;
        frame.extended = msg.extd != 0;
        frame.rtr = msg.rtr != 0;
        frame.dlc = msg.data_length_code > 8 ? 8 : (uint8_t)msg.data_length_code;
        memcpy(frame.data, msg.data, sizeof(frame.data));
        frame.timestamp_ms = millis();

        _stats.rx++;
        _stats.lastRxMs = frame.timestamp_ms;

        if (_handler) _handler(frame);
    }
}

// ─── 总线健康看门狗 ──────────────────────────────────────────

void CanBus::_watchdog() {
    const uint32_t now = millis();
    if (now - _lastWatchdogMs < 100) return;
    _lastWatchdogMs = now;

    twai_status_info_t status;
    if (twai_get_status_info(&status) != ESP_OK) return;

    _stats.tec = status.tx_error_counter;
    _stats.rec = status.rx_error_counter;
    _stats.rxMissed = status.rx_missed_count;

    if (status.state == TWAI_STATE_BUS_OFF) {
        _stats.busOff++;
        Serial.printf("[CAN] 总线 bus-off（TEC=%lu），开始自动恢复\n",
                      (unsigned long)status.tx_error_counter);
        twai_initiate_recovery();
    } else if (status.state == TWAI_STATE_STOPPED) {
        if (twai_start() == ESP_OK) Serial.println("[CAN] 控制器已重新启动");
    }
}

// ─── 统计 ────────────────────────────────────────────────────

void CanBus::getStats(Stats& out) const {
    out = _stats;
    if (!_running) return;

    twai_status_info_t status;
    if (twai_get_status_info(&status) == ESP_OK) {
        out.tec = status.tx_error_counter;
        out.rec = status.rx_error_counter;
        out.rxMissed = status.rx_missed_count;
    }
}

// ─── 时序配置 ────────────────────────────────────────────────

twai_timing_config_t CanBus::_timingConfig(uint32_t baudrate) const {
    // 注意：TWAI 时序宏是复合字面量，不能直接放进三元表达式
    switch (baudrate) {
        case 125000:   return TWAI_TIMING_CONFIG_125KBITS();
        case 250000:   return TWAI_TIMING_CONFIG_250KBITS();
        case 800000:   return TWAI_TIMING_CONFIG_800KBITS();
        case 1000000:  return TWAI_TIMING_CONFIG_1MBITS();
        case 500000:
        default:       return TWAI_TIMING_CONFIG_500KBITS();
    }
}
