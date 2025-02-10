#pragma once

#include <stdexcept>
#include "types/serialPacket.hpp"
#include "types/encodedSerialPacket.hpp"

namespace vexbridge
{
    struct SerialPacketTypes
    {
        static EncodedSerialPacket *serialize(SerialPacket *packet)
        {
            SerialPacketType *packetType = get(packet->type);
            if (packetType == nullptr)
                throw std::runtime_error("Invalid packet type.");

            return packetType->serialize(packet);
        }

        static SerialPacket *deserialize(EncodedSerialPacket *packet)
        {
            SerialPacketType *packetType = get(packet->type);
            if (packetType == nullptr)
                throw std::runtime_error("Invalid packet type.");

            return packetType->deserialize(packet);
        }

        static SerialPacketType *get(SerialPacketTypeID type)
        {
            for (SerialPacketType *packetType : packetTypes)
                if (packetType->type == type)
                    return packetType;
            return nullptr;
        }

        static SerialPacketType *packetTypes[13];
    };
}