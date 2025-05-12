#pragma once

#include <cstdint>
#include <deque>
#include <memory>
#include "../packetTypes/genericAckPacket.hpp"
#include "../serialization/serialPacketWriter.hpp"

namespace vexbridge::serial
{
    struct AckPacketHandler
    {
        /**
         * Checks if a packet is an `GenericAckPacket`.
         * If it is, it will inform the `SerialPacketWriter` to remove the packet from the sent packets list.
         * @param newPacket The packet to handle.
         * @param serialWriter The serial writer to use for writing packets.
         */
        static void handlePacket(SerialPacket *newPacket, SerialPacketWriter *serialWriter)
        {
            // Check if the `newPacket` is nullptr
            if (!newPacket)
                throw std::runtime_error("Cannot handle a nullptr packet.");

            // Check if the `SerialWriter` is nullptr
            if (!serialWriter)
                throw std::runtime_error("Cannot handle a packet with a nullptr serial writer.");

            // Check if the packet is a `GenericAckPacket`
            if (auto ackPacket = dynamic_cast<GenericAckPacket *>(newPacket))
                serialWriter->ackPacket(ackPacket->id);
        }
    };
}