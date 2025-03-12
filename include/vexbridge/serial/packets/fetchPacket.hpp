#pragma once

#include <cstdint>
#include "../types/serialPacket.hpp"
#include "../types/serialPacketType.hpp"
#include "../../utils/bufferWriter.hpp"
#include "../../utils/bufferReader.hpp"

namespace vexbridge::serial
{
    struct FetchPacket : public SerialPacket
    {
    };

    struct FetchPacketType : public SerialPacketType
    {
        FetchPacketType() : SerialPacketType(SerialPacketTypeID::FETCH_VALUES)
        {
        }

        SerialPacket *deserialize(EncodedSerialPacket *packet) override
        {
            FetchPacket *newPacket = new FetchPacket();
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