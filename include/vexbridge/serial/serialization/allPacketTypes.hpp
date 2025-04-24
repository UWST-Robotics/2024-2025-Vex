#pragma once

#include "../packetTypes/common/serialPacketType.h"
#include "../packetTypes/assignLabelPacket.hpp"
#include "../packetTypes/updateIntPacket.hpp"
#include "../packetTypes/updateFloatPacket.hpp"
#include "../packetTypes/updateDoublePacket.hpp"
#include "../packetTypes/updateStringPacket.hpp"
#include "../packetTypes/genericAckPacket.hpp"
#include "../packetTypes/genericNAckPacket.hpp"
#include "../packetTypes/logPacket.hpp"
#include "../packetTypes/pingPacket.hpp"
#include "../packetTypes/resetPacket.hpp"
#include "../packetTypes/updateBoolArrayPacket.hpp"
#include "../packetTypes/updateIntArrayPacket.hpp"
#include "../packetTypes/updateFloatArrayPacket.hpp"
#include "../packetTypes/updateDoubleArrayPacket.hpp"
#include "../packetTypes/updateFloatArrayPacket.hpp"

namespace vexbridge::serial
{
    /**
     * Container for all SerialPacketType objects.
     */
    struct AllPacketTypes
    {
        /**
         * Gets a packet type by it's type ID.
         * @param typeID The type ID of the packet type to get.
         * @return The packet type or nullptr if not found.
         */
        static SerialPacketType *get(const SerialPacketTypeID typeID)
        {
            // Calculate Length of Packet Types
            size_t length = sizeof(ALL_PACKET_TYPES) / sizeof(ALL_PACKET_TYPES[0]);

            // Iterate through all packet types
            for (size_t i = 0; i < length; i++)
            {
                // Check if the packet type matches
                if (ALL_PACKET_TYPES[i]->typeID == typeID)
                    return ALL_PACKET_TYPES[i];
            }

            // Packet type not found
            return nullptr;
        }

        /// @brief Array containing all SerialPacketType objects
        static SerialPacketType *ALL_PACKET_TYPES[14];
    };
}

// Assign Packet Types
vexbridge::serial::SerialPacketType *vexbridge::serial::AllPacketTypes::ALL_PACKET_TYPES[] = {
    new AssignLabelPacketType(),
    new UpdateIntPacketType(),
    new UpdateFloatPacketType(),
    new UpdateDoublePacketType(),
    new UpdateStringPacketType(),
    new GenericAckPacketType(),
    new GenericNAckPacketType(),
    new LogPacketType(),
    new PingPacketType(),
    new ResetPacketType(),
    new UpdateBoolArrayPacketType(),
    new UpdateIntArrayPacketType(),
    new UpdateFloatArrayPacketType(),
    new UpdateDoubleArrayPacketType()};
