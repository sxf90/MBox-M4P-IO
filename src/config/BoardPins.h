#pragma once
#include <stdint.h>

// ════════════════════════════════════════════════════════════════
// MBox-M4P-IO 引脚与编译期参数总表
// 所有与硬件相关的定义集中在本文件：换板子 / 改接线只动这里，
// 业务代码（app/、input/、can/）不需要修改。
// ════════════════════════════════════════════════════════════════

// ─── CAN：SN65HVD230 3.3V 模块（接法与 VoiceBox 完全一致）─────────
//   模块 3.3U            -> 3V3     （严禁接 5V）
//   模块 GND             -> GND     （必须共地）
//   模块 CAN_TX(D 输入)  -> GPIO5   （ESP32 TX）
//   模块 CAN_RX(R 输出)  -> GPIO4   （ESP32 RX）
//   模块 CANH/CANL       -> 总线    （板载 120Ω，量 CANH-CANL 应为 60/120Ω）
#define CAN_TX_PIN      5
#define CAN_RX_PIN      4

// 波特率：125000 / 250000 / 500000 / 800000 / 1000000
#ifndef CAN_BAUD
#define CAN_BAUD        500000
#endif

// 0 = NORMAL       正常收发，总线上必须有其它节点回 ACK（接真实设备）
// 1 = NO_ACK       发送不需要 ACK，单机台架自测用
// 2 = LISTEN_ONLY  只听，不发送也不 ACK
#ifndef CAN_MODE
#define CAN_MODE        0
#endif

// ─── 5 路独立按键（按键板 S1~S5 + COM，S6~S8 不接线）────────────
//   按键板 S1~S5 -> 下表 GPIO
//   按键板 COM   -> GND
//   按键板不需要 VCC（纯机械开关）
//   逻辑：内部上拉，按下 = LOW，松开 = HIGH
//
//   业务：5 个键 = MBox 流媒体页面 0x41~0x45 直接点选（见 app/KeyCanMap.h）
//   页面是绝对寻址（按哪个到哪个），所以键数 = 页面数，不需要翻页/状态记忆。
#define KEY_COUNT       5

// 1 = COM 接 GND，使用内部上拉（默认，按下为 LOW）
// 0 = COM 接 3V3，使用内部下拉（按下为 HIGH）
#ifndef KEY_COM_TO_GND
#define KEY_COM_TO_GND  1
#endif

// 机械按键抖动 5~20ms，建议 20~30ms
#ifndef KEY_DEBOUNCE_MS
#define KEY_DEBOUNCE_MS 25
#endif

// 1 = 引脚探测模式：只打印按键落在哪个 GPIO，不发送 CAN（首次核对接线用）
#ifndef KEY_PIN_SCAN
#define KEY_PIN_SCAN    0
#endif

// 按键 GPIO 分配（顺序即按键板 S1..S5）
// 选型理由：均带内部上拉、非 strapping 脚、不占 Flash(6-11)/UART0(1,3)/CAN(4,5)
// 注意：GPIO22 在 platformio.ini 里被预留给了 WS2812 灯带（本工程不启用）；
//       GPIO23 已分配给蜂鸣器（见文件末尾 BUZZER_PIN），S1~S5 引脚维持不变。
#define KEY_PIN_S1      13
#define KEY_PIN_S2      14
#define KEY_PIN_S3      18
#define KEY_PIN_S4      19
#define KEY_PIN_S5      21
// S6~S8（GPIO22/23/25）硬件不接线，固件不再扫描

static const uint8_t KEY_PINS[KEY_COUNT] = {
    KEY_PIN_S1, KEY_PIN_S2, KEY_PIN_S3, KEY_PIN_S4, KEY_PIN_S5
};

// ─── 0x24「流媒体显示切换」帧发送策略 ────────────────────────────
// 按一下连发几帧：页面值是幂等的，重复帧无副作用，只用来防单帧丢失
#ifndef KEY_TX_REPEAT
#define KEY_TX_REPEAT            3
#endif
// 补发帧间隔（ms）
#ifndef KEY_TX_REPEAT_GAP_MS
#define KEY_TX_REPEAT_GAP_MS     20
#endif
// 同一个页面键连按抑制：这段时间内重复按只算一次（机械抖动/手指重按）
#ifndef KEY_PAGE_MUTEX_MS
#define KEY_PAGE_MUTEX_MS        300
#endif
// 0x24 空闲保活周期：0 = 关闭（默认，只在按键时发送）
// 若 MBox 侧要求 0x24 必须按协议 100ms 周期在线，把这里改成 100 即可，
// 业务代码不用动（保活帧 B3=0x00，协议定义为「保持当前显示状态」）
#ifndef CAN_CTRL_KEEPALIVE_MS
#define CAN_CTRL_KEEPALIVE_MS    0
#endif

// ════════════════════════════════════════════════════════════════
// ─── 蜂鸣器：监听 0x15 的 FCW / RCW 预警（独立业务，与按键无关）──
// ════════════════════════════════════════════════════════════════
// 硬件：5V 有源蜂鸣器模块（高电平触发，模块自带三极管驱动）
//   模块 VCC -> VIN / 5V（USB 5V；不要把 5V 模块接到 3V3）
//   模块 GND -> GND    （与 CAN 模块、按键板 COM 共地）
//   模块 IO  -> GPIO23 （串 100Ω 限流，可选 100kΩ 下拉防上电误响）
//
// 为什么是 GPIO23：本工程 S1~S5 占用 13/14/18/19/21（GPIO19 是 S4，保持不动），
// GPIO23 空闲、非 strapping、可输出，原注释里就是 S6 的预留脚。
//
// 逻辑（与 VoiceBox 的 BuzzerManager 完全一致）：
//   FCW 触发 → 持续高电平长鸣；RCW 触发 → 150ms 响 / 150ms 停 循环
//   FCW 优先于 RCW；预警消失时补完当前这一拍再停
#ifndef BUZZER_PIN
#define BUZZER_PIN        23
#endif

// 1 = 高电平触发（默认，配套常见 5V 有源蜂鸣器模块）
// 0 = 低电平触发（若用低电平触发模块，改这里，不用动业务代码）
#ifndef BUZZER_ACTIVE_HIGH
#define BUZZER_ACTIVE_HIGH 1
#endif

// 0x15 预警链路超时：超过该时间没收到 0x15，视为预警全部消失并强制静音
// （防止 MBox 掉电/断线时正在长鸣的蜂鸣器一直叫）
// 500ms ≈ 容忍连续丢 5 帧（协议周期 100ms）；设为 0 则关闭，行为与 VoiceBox 一致
#ifndef ALARM_LINK_TIMEOUT_MS
#define ALARM_LINK_TIMEOUT_MS  500
#endif

// 1 = 0x15 每帧都打印原始数据（现场核对位序时用，量产设为 0）
#ifndef ALARM_DEBUG_RAW
#define ALARM_DEBUG_RAW   0
#endif

