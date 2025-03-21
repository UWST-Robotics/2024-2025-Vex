#pragma once

#include <cstdint>
#include <string>
#include "common/serialPacket.h"
#include "common/serialPacketType.h"
#include "common/encodedSerialPacket.h"
#include "../../utils/bufferWriter.hpp"
#include "../../utils/bufferReader.hpp"

using namespace vexbridge::utils;

namespace vexbridge::serial
{
    struct AssignLabelPacket : public SerialPacket
    {
        uint16_t ntID;
        std::string label;
    };

    struct AssignLabelPacketType : public SerialPacketType
    {
        AssignLabelPacketType() : SerialPacketType(SerialPacketTypeID::UPDATE_LABEL)
        {
        }

        std::unique_ptr<SerialPacket> deserialize(const EncodedSerialPacket &packet) override
        {
            // Make new update value packet
            auto newPacket = std::make_unique<AssignLabelPacket>();
            newPacket->type = packet.type;
            newPacket->id = packet.id;

            // Read packet contents from payload
            BufferReader reader(packet.payload);
            newPacket->ntID = reader.readUInt16BE();
            newPacket->label = reader.readString8();
            return newPacket;
        }

        std::unique_ptr<EncodedSerialPacket> serialize(const SerialPacket &packet) override
        {
            // Cast packet to update value packet
            const AssignLabelPacket &updateLabelPacket = dynamic_cast<const AssignLabelPacket &>(packet);

            // Allocate buffer for payload
            Buffer payload;
            BufferWriter writer(payload);

            // Write value
            writer.writeUInt16BE(updateLabelPacket.ntID);
            writer.writeString8(updateLabelPacket.label);

            // Make new encoded packet
            auto newPacket = std::make_unique<EncodedSerialPacket>();
            newPacket->type = packet.type;
            newPacket->id = packet.id;
            newPacket->payload = payload;
            return newPacket;
        }
    };
}