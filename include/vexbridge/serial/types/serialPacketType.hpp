#pragma once

#include <cstdint>
#include "serialPacketTypeID.hpp"
#include "serialPacket.hpp"
#include "encodedSerialPacket.hpp"

namespace vexbridge
{
    /**
     * Represents a singular packet of data to be sent over a serial port.
     */
    struct SerialPacketType
    {
        SerialPacketType(SerialPacketTypeID type)
            : type(type)
        {
        }

        SerialPacketTypeID type;
        virtual SerialPacket *deserialize(EncodedSerialPacket *packet) = 0;
        virtual EncodedSerialPacket *serialize(SerialPacket *packet) = 0;
    };
}