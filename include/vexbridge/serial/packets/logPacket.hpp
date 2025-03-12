#pragma once

#include <cstdint>
#include <string>
#include "../types/serialPacket.hpp"
#include "../types/serialPacketType.hpp"
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

        SerialPacket *deserialize(EncodedSerialPacket *packet) override
        {
            BufferReader reader(packet->payload, packet->payloadSize);
            LogPacket *newPacket = new LogPacket();
            newPacket->type = packet->type;
            newPacket->id = packet->id;
            newPacket->message = reader.readString16();
            return newPacket;
        }

        EncodedSerialPacket *serialize(SerialPacket *packet) override
        {
            LogPacket *logPacket = dynamic_cast<LogPacket *>(packet);
            if (logPacket == nullptr)
                return nullptr;

            size_t payloadSize = 2 + logPacket->message.length();
            uint8_t *payload = new uint8_t[payloadSize];
            BufferWriter writer(payload, payloadSize);

            writer.writeString16(logPacket->message);

            return EncodedSerialPacket::build(packet, payload, writer.getOffset());
        }
    };
}