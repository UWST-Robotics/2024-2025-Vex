#pragma once
#include <cstdint>

namespace vexbridge
{
    enum class SerialPacketTypeID : uint8_t
    {
        UNKNOWN = 0x00,
        RESET = 0x01,
        UPDATE_LABEL = 0x02,
        UPDATE_VALUE = 0x03,
        BATCH_VALUE = 0x04,
        LOG = 0x04,
        GENERIC_ACK = 0x11,
        PING = 0x12
    };
}