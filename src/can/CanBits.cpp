#include "CanBits.h"
#include <stddef.h>

namespace canbits {

bool extract(const uint8_t* data, uint8_t dataLen, uint16_t startBit,
             uint8_t bitLength, ByteOrder order, uint32_t& value) {
    value = 0;
    if (!data || bitLength == 0 || bitLength > 32 || dataLen == 0) return false;

    const size_t totalBits = (size_t)dataLen * 8;

    if (order == ByteOrder::INTEL) {
        // Intel：startBit 是最低位，逐位向高位取
        if ((size_t)startBit + bitLength > totalBits) return false;
        for (uint8_t i = 0; i < bitLength; ++i) {
            const size_t bitPos = (size_t)startBit + i;
            const size_t byteIndex = bitPos / 8;
            const uint8_t bitIndex = (uint8_t)(bitPos % 8);
            if ((data[byteIndex] >> bitIndex) & 0x01) value |= (1u << i);
        }
        return true;
    }

    // Motorola：startBit 是最高位，逐位向低位取
    if ((size_t)startBit >= totalBits ||
        (size_t)startBit + 1 < bitLength) return false;
    for (uint8_t i = 0; i < bitLength; ++i) {
        const size_t bitPos = (size_t)startBit - i;
        const size_t byteIndex = bitPos / 8;
        const uint8_t bitIndex = (uint8_t)(bitPos % 8);
        if ((data[byteIndex] >> bitIndex) & 0x01)
            value |= (1u << (bitLength - 1 - i));
    }
    return true;
}

bool bit(const uint8_t* data, uint8_t dataLen, uint16_t bitPos) {
    if (!data || (size_t)bitPos >= (size_t)dataLen * 8) return false;
    return ((data[bitPos / 8] >> (bitPos % 8)) & 0x01) != 0;
}

bool readU8(const uint8_t* data, uint8_t dataLen, uint8_t byteOffset, uint8_t& value) {
    value = 0;
    if (!data || (size_t)byteOffset + 1 > dataLen) return false;
    value = data[byteOffset];
    return true;
}

bool readU16(const uint8_t* data, uint8_t dataLen, uint8_t byteOffset,
             ByteOrder order, uint16_t& value) {
    value = 0;
    if (!data || (size_t)byteOffset + 2 > dataLen) return false;
    if (order == ByteOrder::INTEL)
        value = (uint16_t)(data[byteOffset] | ((uint16_t)data[byteOffset + 1] << 8));
    else
        value = (uint16_t)(((uint16_t)data[byteOffset] << 8) | data[byteOffset + 1]);
    return true;
}

bool readU32(const uint8_t* data, uint8_t dataLen, uint8_t byteOffset,
             ByteOrder order, uint32_t& value) {
    value = 0;
    if (!data || (size_t)byteOffset + 4 > dataLen) return false;
    if (order == ByteOrder::INTEL) {
        value = (uint32_t)data[byteOffset]
              | ((uint32_t)data[byteOffset + 1] << 8)
              | ((uint32_t)data[byteOffset + 2] << 16)
              | ((uint32_t)data[byteOffset + 3] << 24);
    } else {
        value = ((uint32_t)data[byteOffset] << 24)
              | ((uint32_t)data[byteOffset + 1] << 16)
              | ((uint32_t)data[byteOffset + 2] << 8)
              | (uint32_t)data[byteOffset + 3];
    }
    return true;
}

int32_t signExtend(uint32_t raw, uint8_t bitLength) {
    if (bitLength == 0) return 0;
    if (bitLength >= 32) return (int32_t)raw;
    const uint32_t signMask = 1u << (bitLength - 1);
    if (raw & signMask) raw |= ~((1UL << bitLength) - 1);
    return (int32_t)raw;
}

}  // namespace canbits
