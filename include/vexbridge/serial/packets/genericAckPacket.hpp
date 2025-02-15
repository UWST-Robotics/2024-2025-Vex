#pragma once

#include <cstdint>
#include "../types/serialPacket.hpp"
#include "../types/serialPacketType.hpp"
#include "../../utils/bufferWriter.hpp"
#include "../../utils/bufferReader.hpp"

namespace vexbridge
{
    struct GenericAckPacket : public SerialPacket
    {
    };

    struct GenericAckPacketType : public SerialPacketType
    {
        GenericAckPacketType()
            : SerialPacketType(SerialPacketTypeID::GENERIC_ACK)
        {
        }

        SerialPacket *deserialize(EncodedSerialPacket *packet) override
        {
            GenericAckPacket *newPacket = new GenericAckPacket();
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