#pragma once

#include <cstdint>
#include "serialPacketTypeID.hpp"

namespace vexbridge::serial
{
    /**
     * Represents a packet with it's payload deserialized.
     */
    struct SerialPacket
    {
        virtual ~SerialPacket() = default;

        SerialPacketTypeID type = SerialPacketTypeID::UNKNOWN;
        uint8_t id = 0;
    };
}