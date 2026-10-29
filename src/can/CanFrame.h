#pragma once
#include <Arduino.h>
#include <functional>

// 一帧 CAN 报文（与硬件/业务无关的纯数据结构）
struct CanFrame {
    uint32_t id = 0;            // 11bit 标准帧 ID 或 29bit 扩展帧 ID
    bool     extended = false;  // true = 扩展帧（29bit）
    bool     rtr = false;       // true = 远程帧（无数据）
    uint8_t  dlc = 0;           // 数据长度 0~8
    uint8_t  data[8] = {};      // 数据（不足 8 字节的部分补 0）
    uint32_t timestamp_ms = 0;  // 收到该帧的 millis()
};

// 收到报文时的回调（在 canBus.poll() 中调用）
using CanFrameHandler = std::function<void(const CanFrame&)>;
