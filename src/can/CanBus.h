#pragma once
#include <Arduino.h>
#include <functional>
#include "driver/twai.h"
#include "CanFrame.h"
#include "../config/BoardPins.h"

// ESP32 内置 TWAI(CAN) 控制器封装
// 只负责：安装/启动/停止驱动、非阻塞收发、bus-off 自动恢复、收发统计。
// 不含任何客户配置或协议业务 —— 报文怎么解释由 app/ 层决定。
class CanBus {
public:
    // 工作模式（对应 twai_mode_t）
    enum class Mode : uint8_t {
        NORMAL = 0,       // 正常收发：总线上必须有其它节点回 ACK
        NO_ACK = 1,       // 发送不需要 ACK：单机台架自测
        LISTEN_ONLY = 2   // 只听：不发送、不 ACK，不影响总线
    };

    struct Config {
        gpio_num_t tx = (gpio_num_t)CAN_TX_PIN;
        gpio_num_t rx = (gpio_num_t)CAN_RX_PIN;
        uint32_t   baudrate = CAN_BAUD;
        Mode       mode = static_cast<Mode>(CAN_MODE);
        uint8_t    tx_queue_len = 8;    // 默认值只有 5，连按几下就会满
        uint8_t    rx_queue_len = 32;
        uint8_t    max_frames_per_poll = 16;
    };

    struct Stats {
        uint32_t rx = 0;          // 累计接收帧数
        uint32_t tx = 0;          // 累计发送成功帧数
        uint32_t txFailed = 0;    // 累计发送失败（队列满 / 无 ACK 等）
        uint32_t busOff = 0;      // 累计进入 bus-off 次数
        uint32_t rxMissed = 0;    // 接收队列溢出丢失帧数
        uint32_t tec = 0;         // 发送错误计数器
        uint32_t rec = 0;         // 接收错误计数器
        uint32_t lastRxMs = 0;    // 最近一次收到报文的时间
    };

    bool begin(const Config& cfg);
    bool begin();   // 使用 Config 默认值（即 BoardPins.h 的引脚/波特率/模式）
    void stop();

    // loop() 中调用：排空接收队列 + 检测/恢复 bus-off
    void poll();

    // 非阻塞发送（不等待总线空闲），失败只计数不阻塞
    // selfTest = true 时请求自发自收，便于 NO_ACK 模式台架验证链路
    bool transmit(uint32_t id, const uint8_t* data, uint8_t dlc,
                  bool extended = false, bool selfTest = false);

    // 注册接收回调（可选，不注册也能用，接收队列照常排空）
    void onFrame(CanFrameHandler handler) { _handler = handler; }

    void getStats(Stats& out) const;

    bool isRunning() const { return _running; }

    static const char* modeName(Mode mode);

private:
    void _drainRx();
    void _watchdog();
    twai_timing_config_t _timingConfig(uint32_t baudrate) const;

    Config           _cfg;
    CanFrameHandler  _handler;
    Stats            _stats;
    bool             _running = false;
    uint32_t         _lastWatchdogMs = 0;
};

extern CanBus canBus;
