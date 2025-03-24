#pragma once

#include <cstdint>
#include <string>
#include "serialSocket.hpp"
#include "packetTypes/updateBoolPacket.hpp"
#include "packetTypes/updateIntPacket.hpp"
#include "packetTypes/updateFloatPacket.hpp"
#include "packetTypes/updateDoublePacket.hpp"
#include "packetTypes/assignLabelPacket.hpp"

namespace vexbridge::serial
{
    /**
     * Writes data to all active serial sockets.
     */
    struct SerialWriter
    {
        // Prevent instantiation
        SerialWriter() = delete;

        static void updateBoolean(uint16_t id, bool value)
        {
            auto packet = std::make_shared<UpdateBoolPacket>();
            packet->id = id;
            packet->valueID = id;
            packet->newValue = value;
            SerialSocket::writePacketToAll(packet);
        }

        static void updateInt(uint16_t id, int32_t value)
        {
            auto packet = std::make_shared<UpdateIntPacket>();
            packet->id = id;
            packet->valueID = id;
            packet->newValue = value;
            SerialSocket::writePacketToAll(packet);
        }

        static void updateFloat(uint16_t id, float value)
        {
            auto packet = std::make_shared<UpdateFloatPacket>();
            packet->id = id;
            packet->valueID = id;
            packet->newValue = value;
            SerialSocket::writePacketToAll(packet);
        }

        static void updateDouble(uint16_t id, double value)
        {
            auto packet = std::make_shared<UpdateDoublePacket>();
            packet->id = id;
            packet->valueID = id;
            packet->newValue = value;
            SerialSocket::writePacketToAll(packet);
        }

        static void assignLabel(uint16_t id, std::string label)
        {
            auto packet = std::make_shared<AssignLabelPacket>();
            packet->id = id;
            packet->valueID = id;
            packet->label = label;
            SerialSocket::writePacketToAll(packet);
        }
    };
}