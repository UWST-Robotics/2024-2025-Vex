#pragma once

#include "packets/resetPacket.hpp"
#include "packets/updateLabelPacket.hpp"
#include "packets/updateValuePacket.hpp"
#include "packets/genericAckPacket.hpp"

namespace vexbridge
{
    struct SerialPacketTypes
    {
        static EncodedSerialPacket *serialize(SerialPacket *packet)
        {
            SerialPacketType *packetType = getPacketType(packet->type);
            if (packetType == nullptr)
                return nullptr;

            return packetType->serialize(packet);
        }

        static SerialPacket *deserialize(EncodedSerialPacket *packet)
        {
            SerialPacketType *packetType = getPacketType(packet->type);
            if (packetType == nullptr)
                return nullptr;

            return packetType->deserialize(packet);
        }

        static SerialPacketType *getPacketType(SerialPacketTypeID type)
        {
            for (SerialPacketType *packetType : packetTypes)
                if (packetType->type == type)
                    return packetType;
            return nullptr;
        }

        static SerialPacketType *packetTypes[4];
    };
}

// All Packet Types
vexbridge::SerialPacketType *vexbridge::SerialPacketTypes::packetTypes[4] = {
    new ResetPacketType(),
    new UpdateLabelPacketType(),
    new UpdateValuePacketType(),
    new GenericAckPacketType()};