#pragma once

#include <cstdint>
#include <memory>
#include "serialPacketTypeID.h"
#include "serialPacket.h"
#include "encodedSerialPacket.h"

namespace vexbridge::serial
{
    /**
     * Represents a singular packet of data to be sent over a serial port.
     */
    struct SerialPacketType
    {
        SerialPacketType(const SerialPacketTypeID typeID)
            : typeID(typeID)
        {
        }

        // Required for inheritance
        virtual ~SerialPacketType() = default;

        const SerialPacketTypeID typeID;
        virtual std::unique_ptr<SerialPacket> deserialize(const EncodedSerialPacket &packet) = 0;
        virtual std::unique_ptr<EncodedSerialPacket> serialize(const SerialPacket &packet) = 0;
    };
}