#pragma once

#include <cstdint>
#include "common/serialPacket.h"
#include "common/serialPacketType.h"
#include "common/encodedSerialPacket.h"
#include "../../utils/bufferWriter.hpp"
#include "../../utils/bufferReader.hpp"

namespace vexbridge::serial
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

        std::unique_ptr<SerialPacket> deserialize(const EncodedSerialPacket &packet) override
        {
            // Make new ack packet
            auto newPacket = std::make_unique<GenericAckPacket>();
            newPacket->type = packet.type;
            newPacket->id = packet.id;
            return newPacket;
        }

        std::unique_ptr<EncodedSerialPacket> serialize(const SerialPacket &packet) override
        {
            // Make new encoded packet
            auto newPacket = std::make_unique<EncodedSerialPacket>();
            newPacket->type = packet.type;
            newPacket->id = packet.id;
            return newPacket;
        }
    };
}