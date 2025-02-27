#pragma once

#include <cstdint>
#include "../types/serialPacket.hpp"
#include "../types/serialPacketType.hpp"
#include "../../utils/bufferWriter.hpp"
#include "../../utils/bufferReader.hpp"

namespace vexbridge
{
    struct GenericNAckPacket : public SerialPacket
    {
    };

    struct GenericNAckPacketType : public SerialPacketType
    {
        GenericNAckPacketType()
            : SerialPacketType(SerialPacketTypeID::GENERIC_NACK)
        {
        }

        SerialPacket *deserialize(EncodedSerialPacket *packet) override
        {
            GenericNAckPacket *newPacket = new GenericNAckPacket();
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