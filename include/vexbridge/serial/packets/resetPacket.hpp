#pragma once

#include <cstdint>
#include "../types/serialPacket.hpp"
#include "../types/serialPacketType.hpp"
#include "../../utils/bufferWriter.hpp"
#include "../../utils/bufferReader.hpp"

namespace vexbridge
{
    struct ResetPacket : public SerialPacket
    {
    };

    struct ResetPacketType : public SerialPacketType
    {
        ResetPacketType() : SerialPacketType(SerialPacketTypeID::RESET)
        {
        }

        SerialPacket *deserialize(EncodedSerialPacket *packet) override
        {
            BufferReader reader(packet->payload, packet->payloadSize);
            ResetPacket *newPacket = new ResetPacket();
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