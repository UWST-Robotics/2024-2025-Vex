#pragma once

#include <cstdint>
#include "../types/serialPacket.hpp"
#include "../types/serialPacketType.hpp"
#include "../../utils/bufferWriter.hpp"
#include "../../utils/bufferReader.hpp"

namespace vexbridge::serial
{
    struct PingPacket : public SerialPacket
    {
    };

    struct PingPacketType : public SerialPacketType
    {
        PingPacketType() : SerialPacketType(SerialPacketTypeID::PING)
        {
        }

        SerialPacket *deserialize(EncodedSerialPacket *packet) override
        {
            PingPacket *newPacket = new PingPacket();
            newPacket->type = packet->type;
            newPacket->id = packet->id;
            return newPacket;
        }

        EncodedSerialPacket *serialize(SerialPacket *packet) override
        {
            return EncodedSerialPacket::build(packet, nullptr, 0);
        }
    };
}