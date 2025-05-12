#pragma once

#include <cstdint>
#include "serialPacketTypeID.h"

namespace vexbridge::serial
{
    /**
     * Represents a packet with it's payload deserialized.
     */
    struct SerialPacket
    {
        // Required for inheritance
        virtual ~SerialPacket() = default;

        SerialPacketTypeID type = SerialPacketTypeID::UNKNOWN;
        uint8_t id = 0;
    };
}