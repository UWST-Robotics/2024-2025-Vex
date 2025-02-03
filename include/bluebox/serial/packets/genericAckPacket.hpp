#pragma once

#include <cstdint>
#include "../types/serialPacket.hpp"
#include "../types/serialPacketType.hpp"
#include "../../utils/bufferWriter.hpp"
#include "../../utils/bufferReader.hpp"

namespace bluebox
{
    struct GenericAckPacket : public SerialPacket
    {
        uint8_t targetID = 0;
    };

    struct GenericAckPacketType : public SerialPacketType
    {
        GenericAckPacketType()
            : SerialPacketType(SerialPacketTypeID::GENERIC_ACK)
        {
        }

        SerialPacket *deserialize(EncodedSerialPacket *packet) override
        {
            BufferReader reader(packet->payload, packet->payloadSize);
            GenericAckPacket *newPacket = new GenericAckPacket();
            newPacket->type = packet->type;
            newPacket->id = packet->id;
            newPacket->targetID = reader.readUInt8();
            return newPacket;
        }

        EncodedSerialPacket *serialize(SerialPacket *packet) override
        {
            GenericAckPacket *genericAckPacket = dynamic_cast<GenericAckPacket *>(packet);
            if (genericAckPacket == nullptr)
                return nullptr;

            uint8_t *buffer = new uint8_t[1];
            BufferWriter writer(buffer, 0);
            writer.writeUInt8(genericAckPacket->targetID);

            return EncodedSerialPacket::build(packet, buffer, writer.getOffset());
        }
    };
}