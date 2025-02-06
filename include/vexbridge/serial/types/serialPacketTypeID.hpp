#pragma once
#include <cstdint>

namespace vexbridge
{
    enum class SerialPacketTypeID : uint8_t
    {
        RESET = 0x01,
        UPDATE_LABEL = 0x02,
        UPDATE_VALUE = 0x03,
        LOG = 0x04,
        GENERIC_ACK = 0x11,
        PING = 0x12
    };
}