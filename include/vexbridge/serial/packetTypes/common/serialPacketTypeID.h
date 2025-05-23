#pragma once

#include <cstdint>

namespace vexbridge::serial
{
    enum class SerialPacketTypeID : uint8_t
    {
        UNKNOWN = 0x00,
        RESET = 0x01,
        ASSIGN_LABEL = 0x02,
        FETCH_VALUES = 0x03,
        LOG = 0x04,
        PING = 0x05,
        GENERIC_ACK = 0x06,
        GENERIC_NACK = 0x07,

        UPDATE_BOOL = 0x21,
        UPDATE_INT = 0x22,
        UPDATE_FLOAT = 0x23,
        UPDATE_DOUBLE = 0x24,
        UPDATE_STRING = 0x25,

        UPDATE_BOOL_ARRAY = 0x31,
        UPDATE_INT_ARRAY = 0x32,
        UPDATE_FLOAT_ARRAY = 0x33,
        UPDATE_DOUBLE_ARRAY = 0x34,

        BATCH_PACKET = 0xFF,
    };
}