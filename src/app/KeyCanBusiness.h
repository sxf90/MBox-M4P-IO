#pragma once
#include <Arduino.h>
#include "../input/KeyPad.h"
#include "KeyCanMap.h"

// 业务层：把按键事件翻译成 CAN 报文并发送。
// 报文内容全部来自 KeyCanMap.h 的表格，本文件不含任何客户协议细节。
//
// 发送语义（针对 0x24 页面帧这类「绝对寻址 + 幂等」的控制信号）：
//   1. 按下：立即发 1 帧，随后按 KEY_TX_REPEAT_GAP_MS 间隔补发到 KEY_TX_REPEAT 帧；
//      补发在 tick() 里非阻塞推进，不用 delay()，不影响按键扫描和其它报文。
//   2. 松开：不发送（PRESS_ONLY）。
//   3. 同一条报文（同 ID + 同帧类型）的按键互斥：新按下的键会取消其它键还没发完的
//      补发帧，避免「旧页面帧晚于新页面帧」导致 MBox 停在错误页面。
//   4. 同一个键 KEY_PAGE_MUTEX_MS 内重复按只算一次。
//   5. CAN_CTRL_KEEPALIVE_MS > 0 时，空闲期按该周期发送全 0 保活帧
//      （B3=0x00 协议定义为「保持当前显示状态」，不会改变 MBox 页面）。
class KeyCanBusiness {
public:
    // pad 用于 HOLD_REPEAT 模式下判断按键是否仍被按住
    void begin(const KeyPad* pad);

    // 绑定到 KeyPad::onEvent
    void onKeyEvent(const KeyEvent& evt);

    // loop() 中调用：处理补发帧、HOLD_REPEAT、保活帧
    void tick();

    // 串口调试：模拟某个按键按下（不需要真实按键板）
    void simulatePress(uint8_t index);

    void printTable() const;

    // 串口统计用：最近一次按下的页面名称（没按过返回提示文字）
    const char* lastPageName() const;

    const KeyAction* lastAction() const;

    bool isRunning() const { return _running; }

private:
    bool _sendAction(uint8_t index, bool pressed);

    // 按下处理：页面互斥 + 立即发 1 帧 + 安排补发（真实按键与串口模拟共用）
    void _press(uint8_t index);

    // 同一条报文里其它键的补发帧作废（页面互斥）
    void _cancelPending(uint8_t exceptIndex);

    const KeyPad* _pad = nullptr;
    bool          _running = false;
    uint32_t      _nextRepeatMs[KeyPad::MAX_KEYS] = {};   // HOLD_REPEAT 下一次发送时间
    uint8_t       _pendingLeft[KeyPad::MAX_KEYS] = {};    // 还要补发几帧
    uint32_t      _pendingNextMs[KeyPad::MAX_KEYS] = {};  // 下一次补发时间
    uint32_t      _lastPressMs[KeyPad::MAX_KEYS] = {};    // 最近一次确认按下（连按抑制用）
    uint8_t       _lastIndex = 0xFF;                      // 最近按下的键（0xFF = 还没按过）
    uint32_t      _lastKeepAliveMs = 0;
};

extern KeyCanBusiness keyCanBusiness;
