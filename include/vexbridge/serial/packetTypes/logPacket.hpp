#pragma once

#include <cstdint>
#include <string>
#include "common/serialPacket.h"
#include "common/serialPacketType.h"
#include "common/encodedSerialPacket.h"
#include "../../utils/bufferWriter.hpp"
#include "../../utils/bufferReader.hpp"

namespace vexbridge::serial
{
    struct LogPacket : public SerialPacket
    {
        std::string message = "";
    };

    struct LogPacketType : public SerialPacketType
    {
        LogPacketType() : SerialPacketType(SerialPacketTypeID::LOG)
        {
        }

        std::unique_ptr<SerialPacket> deserialize(const EncodedSerialPacket &packet) override
        {
            // Make new log packet
            auto newPacket = std::make_unique<LogPacket>();
            newPacket->type = packet.type;
            newPacket->id = packet.id;

            // Read packet contents from payload
            BufferReader reader(packet.payload);
            newPacket->message = reader.readString16();
            return newPacket;
        }

        std::unique_ptr<EncodedSerialPacket> serialize(const SerialPacket &packet) override
        {
            // Cast packet to log packet
            const LogPacket &logPacket = dynamic_cast<const LogPacket &>(packet);

            // Allocate buffer for payload
            Buffer payload;
            BufferWriter writer(payload);

            // Write message
            writer.writeString16(logPacket.message);

            // Make new encoded packet
            auto newPacket = std::make_unique<EncodedSerialPacket>();
            newPacket->type = packet.type;
            newPacket->id = packet.id;
            newPacket->payload = payload;
            return newPacket;
        }
    };
}