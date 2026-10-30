/*
 * MBox-M4P-IO — 5 路按键 → MBox 流媒体页面切换(0x24) + 监听 0x15 的 FCW/RCW 预警蜂鸣
 *
 * 功能：按键板 S1~S5 按下 → 消抖确认 → 通过 SN65HVD230(3.3V) + ESP32 TWAI
 *       发送 0x24 报文里的「流媒体显示切换」信号（B3 = 0x41~0x45），
 *       让 MBox 在 5 种四画面布局之间切换。报文内容见 app/KeyCanMap.h。
 *
 *   5 个键 = 5 个页面，绝对寻址直接点选（按哪个到哪个），
 *   不需要翻页、不需要记忆状态，因此不会出现丢帧后永久错位的问题。
 *   MBox 上电默认就是 0x41，所以上电时本板不发任何帧、不去改动页面。
 *
 * 硬件接线（详见 config/BoardPins.h）：
 *   CAN 模块 3.3U            -> 3V3（严禁 5V）
 *   CAN 模块 GND             -> GND
 *   CAN 模块 CAN_TX(D 输入)  -> GPIO5（ESP32 TX）
 *   CAN 模块 CAN_RX(R 输出)  -> GPIO4（ESP32 RX）
 *   CAN 模块 CANH/CANL       -> 总线（板载 120Ω，500 kbps）
 *   按键板 S1~S5             -> GPIO13/14/18/19/21
 *   按键板 COM               -> GND（按键板不需要 VCC）
 *   按键板 S6~S8             -> 不接线（固件不扫描）
 *
 * 第二条业务（与按键完全独立）：监听 0x15「ADAS 状态广播」里的
 *   FCW（前车碰撞预警，B3.2）与 RCW（后车碰撞预警，B3.0~B3.1），
 *   用蜂鸣器提示：FCW = 持续长鸣；RCW = 150ms 响 / 150ms 停 滴滴滴。
 *   蜂鸣器：5V 有源模块（高电平触发）VCC->VIN/5V，GND->GND，IO->GPIO23。
 *   （S4 仍占用 GPIO19，按键引脚与业务一行未改）
 *
 * 页面（0x24 报文的 B3）：
 *   S1 = 0x41 后向为主+前左右三小图      S2 = 0x42 前向为主+后左右三小图
 *   S3 = 0x43 左侧为主+前后右三小图      S4 = 0x44 右侧为主+前后左三小图
 *   S5 = 0x45 组合页面+四图平铺
 *
 * 串口命令（115200）：
 *   k1~k5  模拟按键按下（不需要按键板即可验证报文）
 *   f      模拟 FCW 触发（长鸣）      r   模拟 RCW 触发（滴滴滴）
 *   x      模拟预警全部消失（验证"补一拍"）  b   打印蜂鸣器/预警状态
 *   p      打印 CAN 统计 + 最近按下的页面 + 预警状态
 *   t      打印按键 → 报文映射表
 */

#include <Arduino.h>
#include "config/BoardPins.h"
#include "can/CanBus.h"
#include "input/KeyPad.h"
#include "app/KeyCanMap.h"
#include "app/KeyCanBusiness.h"
#include "buzzer/Buzzer.h"
#include "app/AlarmBusiness.h"

#ifndef ENABLE_SERIAL_SIM
#define ENABLE_SERIAL_SIM 1
#endif

#ifndef ENABLE_BUZZER
#define ENABLE_BUZZER 1
#endif

static KeyPad keyPad;

#if ENABLE_SERIAL_SIM
static char pendingCmd = 0;

static void printCanStats() {
    CanBus::Stats stats;
    canBus.getStats(stats);

    Serial.printf("[CAN] %s | 接收 %lu 帧 / 发送 %lu 帧 / 失败 %lu 次\n",
                  canBus.isRunning() ? "运行中" : "未启动",
                  (unsigned long)stats.rx, (unsigned long)stats.tx,
                  (unsigned long)stats.txFailed);
    Serial.printf("[CAN] TEC=%lu REC=%lu bus-off=%lu 次 接收丢帧=%lu\n",
                  (unsigned long)stats.tec, (unsigned long)stats.rec,
                  (unsigned long)stats.busOff, (unsigned long)stats.rxMissed);
    if (stats.rx == 0) {
        Serial.println("[CAN] 尚未收到任何报文");
    } else {
        Serial.printf("[CAN] 最近收帧：%lu ms 前\n",
                      (unsigned long)(millis() - stats.lastRxMs));
    }

    const KeyAction* last = keyCanBusiness.lastAction();
    if (last == nullptr) {
        Serial.println("[KEY] 最近页面：（尚未按过按键）");
    } else {
        Serial.printf("[KEY] 最近页面：%s → ID=0x%03lX B3=0x%02X\n", last->name,
                      (unsigned long)last->id, last->data[3]);
    }

#if ENABLE_BUZZER
    alarmBusiness.printStatus();
#endif
}

static void handleSerialCommand() {
    while (Serial.available() > 0) {
        const char c = (char)Serial.read();

        if (pendingCmd == 'k') {
            pendingCmd = 0;
            // '1' ~ '1'+KEY_COUNT-1：按键总数变化时这里自动跟着变
            if (c >= '1' && c < (char)('1' + KEY_COUNT)) {
                keyCanBusiness.simulatePress((uint8_t)(c - '1'));
                continue;
            }
        }
        if (c == 'k' || c == 'K') { pendingCmd = 'k'; continue; }
        if (c == 'p' || c == 'P') { printCanStats(); continue; }
        if (c == 't' || c == 'T') { keyCanBusiness.printTable(); continue; }

#if ENABLE_BUZZER
        // 蜂鸣器调试：不需要 CAN 总线即可验证三种响法
        if (c == 'f' || c == 'F') { alarmBusiness.simulateFCW(true); continue; }
        if (c == 'r' || c == 'R') { alarmBusiness.simulateRCW(true); continue; }
        if (c == 'x' || c == 'X') { alarmBusiness.simulateClearAll(); continue; }
        if (c == 'b' || c == 'B') { alarmBusiness.printStatus(); continue; }
#endif
    }
}
#endif  // ENABLE_SERIAL_SIM

// ─── setup ───────────────────────────────────────────────────

void setup() {
    Serial.begin(115200);
    delay(300);
    Serial.println("\n\n===== MBox-M4P-IO 启动 =====");
    Serial.printf("[App] 芯片: %s  核心: %d  闪存: %lu bytes\n",
                  ESP.getChipModel(), ESP.getChipCores(),
                  (unsigned long)ESP.getFlashChipSize());

    // CAN：波特率 / 模式 / 引脚默认取自 config/BoardPins.h
    CanBus::Config canConfig;
    if (!canBus.begin(canConfig)) {
        Serial.println("[App] CAN 初始化失败：按键报文无法发送，请检查模块接线");
    }

#if KEY_PIN_SCAN
    // 引脚探测模式：只打印按键对应的 GPIO，不发送 CAN
    if (!keyPad.begin(KEY_PINS, KEY_COUNT, KEY_COM_TO_GND != 0)) {
        Serial.println("[App] 按键初始化失败");
    }
    Serial.println("[App] 引脚探测模式：按下按键观察 [SCAN] 日志，核对 S1~S5 接线");
#else
    if (!keyPad.begin(KEY_PINS, KEY_COUNT, KEY_COM_TO_GND != 0)) {
        Serial.println("[App] 按键初始化失败");
    } else {
        keyCanBusiness.begin(&keyPad);
        keyPad.onEvent([](const KeyEvent& evt) { keyCanBusiness.onKeyEvent(evt); });
    }
#endif

#if ENABLE_BUZZER
    // 蜂鸣器业务：监听 0x15 的 FCW/RCW 预警（与按键业务完全独立）
    alarmBusiness.begin();
#endif

#if ENABLE_SERIAL_SIM
    Serial.printf("[App] 串口命令: k1~k%d 模拟按键, p 打印 CAN 统计, t 打印报文表\n",
                  KEY_COUNT);
#if ENABLE_BUZZER
    Serial.println("[App] 蜂鸣器调试: f 模拟FCW, r 模拟RCW, x 模拟全部消失, b 蜂鸣器状态");
#endif
#endif
}

// ─── loop ────────────────────────────────────────────────────

void loop() {
#if KEY_PIN_SCAN
    keyPad.debugScan();
#endif

    canBus.poll();          // 排空接收队列（0x15 预警帧在这里回调）+ bus-off 自动恢复
    keyPad.poll();          // 5ms 节拍扫描 + 消抖
    keyCanBusiness.tick();  // HOLD_REPEAT 周期发送

#if ENABLE_BUZZER
    alarmBusiness.tick();   // 0x15 链路超时检查
    buzzer.poll();          // 推进 RCW 滴滴滴节拍 / 处理"补一拍"
#endif

#if ENABLE_SERIAL_SIM
    handleSerialCommand();
#endif
}
