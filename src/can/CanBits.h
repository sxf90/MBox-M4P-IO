#pragma once
#include <stdint.h>

// CAN 报文位域解析工具（纯函数，不含任何客户/业务语义）
// 供"接收报文并解析信号"的业务使用；只发送按键报文的场景可以不用。
//
// 位编号约定（与 VoiceBox 工程一致）：
//   INTEL(小端)    startBit 为信号最低位(LSB)，向更高位读取
//   MOTOROLA(大端) startBit 为信号最高位(MSB)，向更低位读取
//   Data1 的 bit0 = 0，Data2 的 bit0 = 8，依此类推
namespace canbits {

enum class ByteOrder : uint8_t { INTEL = 0, MOTOROLA = 1 };

// 提取 bitLength 位（1~32）；越界返回 false，value 置 0
bool extract(const uint8_t* data, uint8_t dataLen, uint16_t startBit,
             uint8_t bitLength, ByteOrder order, uint32_t& value);

// 读取单个 bit
bool bit(const uint8_t* data, uint8_t dataLen, uint16_t bitPos);

// 按字节偏移读取无符号整数
bool readU8(const uint8_t* data, uint8_t dataLen, uint8_t byteOffset, uint8_t& value);
bool readU16(const uint8_t* data, uint8_t dataLen, uint8_t byteOffset,
             ByteOrder order, uint16_t& value);
bool readU32(const uint8_t* data, uint8_t dataLen, uint8_t byteOffset,
             ByteOrder order, uint32_t& value);

// 按信号位宽做符号位扩展（bitLength < 32）
int32_t signExtend(uint32_t raw, uint8_t bitLength);

}  // namespace canbits
