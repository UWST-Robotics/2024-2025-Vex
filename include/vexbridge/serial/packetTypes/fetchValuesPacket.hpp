#pragma once

#include <cstdint>
#include "common/serialPacket.h"
#include "common/serialPacketType.h"
#include "common/encodedSerialPacket.h"
#include "../../utils/bufferWriter.hpp"
#include "../../utils/bufferReader.hpp"

namespace vexbridge::serial
{
    struct FetchValuesPacket : public SerialPacket
    {
    };

    struct FetchValuesPacketType : public SerialPacketType
    {
        FetchValuesPacketType() : SerialPacketType(SerialPacketTypeID::FETCH_VALUES)
        {
        }

        std::unique_ptr<SerialPacket> deserialize(const EncodedSerialPacket &packet) override
        {
            // Make new fetch values packet
            auto newPacket = std::make_unique<FetchValuesPacket>();
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