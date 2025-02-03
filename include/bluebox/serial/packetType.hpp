#pragma once
#include <cstdint>

namespace bluebox
{
    enum class PacketType : uint8_t
    {
        // Requests
        RESET = 0x01,
        UPDATE_LABEL = 0x02,
        UPDATE_VALUE = 0x03,

        // Responses
        GENERIC_ACK = 0x51,
    };
}