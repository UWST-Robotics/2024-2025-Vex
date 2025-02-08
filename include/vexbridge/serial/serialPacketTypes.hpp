#pragma once

#include "packets/resetPacket.hpp"
#include "packets/updateLabelPacket.hpp"
#include "packets/updateValuePacket.hpp"
#include "packets/genericAckPacket.hpp"
#include "packets/batchValuePacket.hpp"
#include <stdexcept>

namespace vexbridge
{
    struct SerialPacketTypes
    {
        static EncodedSerialPacket *serialize(SerialPacket *packet)
        {
            SerialPacketType *packetType = getPacketType(packet->type);
            if (packetType == nullptr)
                throw std::runtime_error("Invalid packet type.");

            return packetType->serialize(packet);
        }

        static SerialPacket *deserialize(EncodedSerialPacket *packet)
        {
            SerialPacketType *packetType = getPacketType(packet->type);
            if (packetType == nullptr)
                throw std::runtime_error("Invalid packet type.");

            return packetType->deserialize(packet);
        }

        static SerialPacketType *getPacketType(SerialPacketTypeID type)
        {
            for (SerialPacketType *packetType : packetTypes)
                if (packetType->type == type)
                    return packetType;
            return nullptr;
        }

        static SerialPacketType *packetTypes[5];
    };
}

// All Packet Types
vexbridge::SerialPacketType *vexbridge::SerialPacketTypes::packetTypes[5] = {
    new ResetPacketType(),
    new UpdateLabelPacketType(),
    new UpdateValuePacketType(),
    new GenericAckPacketType(),
    new BatchValuePacketType()};