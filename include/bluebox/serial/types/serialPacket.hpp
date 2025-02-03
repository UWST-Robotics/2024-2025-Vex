#pragma once

#include <cstdint>
#include "serialPacketTypeID.hpp"

namespace bluebox
{
    /**
     * Represents a packet with it's payload deserialized.
     */
    struct SerialPacket
    {
        virtual ~SerialPacket() = default;

        SerialPacketTypeID type;
        uint8_t id;
    };
}