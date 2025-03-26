#pragma once

#include <cstdint>
#include <string>
#include "serialSocket.hpp"
#include "packetTypes/updateBoolPacket.hpp"
#include "packetTypes/updateIntPacket.hpp"
#include "packetTypes/updateFloatPacket.hpp"
#include "packetTypes/updateDoublePacket.hpp"
#include "packetTypes/assignLabelPacket.hpp"
#include "packetTypes/updateBoolArrayPacket.hpp"
#include "packetTypes/updateIntArrayPacket.hpp"
#include "packetTypes/updateFloatArrayPacket.hpp"
#include "packetTypes/updateDoubleArrayPacket.hpp"

namespace vexbridge::serial
{
    /**
     * Writes data to all active serial sockets.
     */
    struct SerialWriter
    {
        // Prevent instantiation
        SerialWriter() = delete;

        static void updateBool(uint16_t id, bool value)
        {
            auto packet = std::make_shared<UpdateBoolPacket>();
            packet->type = SerialPacketTypeID::UPDATE_BOOL;
            packet->valueID = id;
            packet->newValue = value;
            SerialSocket::writePacketToAll(packet);
        }

        static void updateInt(uint16_t id, int32_t value)
        {
            auto packet = std::make_shared<UpdateIntPacket>();
            packet->type = SerialPacketTypeID::UPDATE_INT;
            packet->valueID = id;
            packet->newValue = value;
            SerialSocket::writePacketToAll(packet);
        }

        static void updateFloat(uint16_t id, float value)
        {
            auto packet = std::make_shared<UpdateFloatPacket>();
            packet->type = SerialPacketTypeID::UPDATE_FLOAT;
            packet->valueID = id;
            packet->newValue = value;
            SerialSocket::writePacketToAll(packet);
        }

        static void updateDouble(uint16_t id, double value)
        {
            auto packet = std::make_shared<UpdateDoublePacket>();
            packet->type = SerialPacketTypeID::UPDATE_DOUBLE;
            packet->valueID = id;
            packet->newValue = value;
            SerialSocket::writePacketToAll(packet);
        }

        static void updateString(uint16_t id, std::string value)
        {
            auto packet = std::make_shared<UpdateStringPacket>();
            packet->type = SerialPacketTypeID::UPDATE_STRING;
            packet->valueID = id;
            packet->newValue = value;
            SerialSocket::writePacketToAll(packet);
        }

        static void assignLabel(uint16_t id, std::string label)
        {
            auto packet = std::make_shared<AssignLabelPacket>();
            packet->type = SerialPacketTypeID::ASSIGN_LABEL;
            packet->valueID = id;
            packet->label = label;
            SerialSocket::writePacketToAll(packet);
        }

        static void updateBoolArray(uint16_t id, std::vector<bool> value)
        {
            auto packet = std::make_shared<UpdateBoolArrayPacket>();
            packet->type = SerialPacketTypeID::UPDATE_BOOL_ARRAY;
            packet->valueID = id;
            packet->newValue = value;
            SerialSocket::writePacketToAll(packet);
        }

        static void updateIntArray(uint16_t id, std::vector<int32_t> value)
        {
            auto packet = std::make_shared<UpdateIntArrayPacket>();
            packet->type = SerialPacketTypeID::UPDATE_INT_ARRAY;
            packet->valueID = id;
            packet->newValue = value;
            SerialSocket::writePacketToAll(packet);
        }

        static void updateFloatArray(uint16_t id, std::vector<float> value)
        {
            auto packet = std::make_shared<UpdateFloatArrayPacket>();
            packet->type = SerialPacketTypeID::UPDATE_FLOAT_ARRAY;
            packet->valueID = id;
            packet->newValue = value;
            SerialSocket::writePacketToAll(packet);
        }

        static void updateDoubleArray(uint16_t id, std::vector<double> value)
        {
            auto packet = std::make_shared<UpdateDoubleArrayPacket>();
            packet->type = SerialPacketTypeID::UPDATE_DOUBLE_ARRAY;
            packet->valueID = id;
            packet->newValue = value;
            SerialSocket::writePacketToAll(packet);
        }
    };
}