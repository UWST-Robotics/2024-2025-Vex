#pragma once

#include <cstdint>
#include <string>
#include "../types/serialPacket.hpp"
#include "../types/serialPacketType.hpp"
#include "../../utils/bufferWriter.hpp"
#include "../../utils/bufferReader.hpp"

namespace bluebox
{
    struct UpdateLabelPacket : public SerialPacket
    {
        uint16_t ntID = 0;
        std::string label = "";
    };

    struct UpdateLabelPacketType : public SerialPacketType
    {
        UpdateLabelPacketType()
            : SerialPacketType(SerialPacketTypeID::UPDATE_LABEL)
        {
        }

        SerialPacket *deserialize(EncodedSerialPacket *packet) override
        {
            BufferReader reader(packet->payload, packet->payloadSize);
            UpdateLabelPacket *newPacket = new UpdateLabelPacket();
            newPacket->type = packet->type;
            newPacket->id = packet->id;
            newPacket->ntID = reader.readUInt16LE();
            newPacket->label = reader.readString();
            return newPacket;
        }

        EncodedSerialPacket *serialize(SerialPacket *packet) override
        {
            UpdateLabelPacket *updateLabelPacket = dynamic_cast<UpdateLabelPacket *>(packet);
            if (updateLabelPacket == nullptr)
                return nullptr;

            size_t payloadSize = 4 + updateLabelPacket->label.length();
            uint8_t *payload = new uint8_t[payloadSize];
            BufferWriter writer(payload, payloadSize);

            writer.writeUInt16LE(updateLabelPacket->ntID);
            writer.writeString(updateLabelPacket->label);

            return EncodedSerialPacket::build(packet, payload, writer.getOffset());
        }
    };
}