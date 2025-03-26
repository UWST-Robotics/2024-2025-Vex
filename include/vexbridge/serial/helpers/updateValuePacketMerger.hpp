#pragma once

#include <cstdint>
#include <deque>
#include <memory>
#include "../packetTypes/updateValuePacket.hpp"
#include "../packetTypes/updateBoolPacket.hpp"
#include "../packetTypes/updateIntPacket.hpp"
#include "../packetTypes/updateFloatPacket.hpp"
#include "../packetTypes/updateDoublePacket.hpp"
#include "../packetTypes/updateStringPacket.hpp"
#include "../packetTypes/updateBoolArrayPacket.hpp"
#include "../packetTypes/updateIntArrayPacket.hpp"
#include "../packetTypes/updateFloatArrayPacket.hpp"
#include "../packetTypes/updateDoubleArrayPacket.hpp"

namespace vexbridge::serial
{
    struct UpdateValuePacketMerger
    {
        /**
         * Checks if a packet is an `UpdateValuePacket`.
         * If it is, it will replace existing packets in the queue with the same ID.
         * If not, it will ignore the packet.
         * @param queue The queue of packets to merge into.
         * @param newPacket The new packet to merge.
         * @return `true` if the packet was merged, `false` if the packet was ignored.
         */
        static bool mergeInQueue(
            std::deque<std::shared_ptr<SerialPacket>> &queue,
            std::shared_ptr<SerialPacket> newPacket)
        {
            // Try to get the value ID
            auto valueID = tryGetValueID(newPacket);
            if (valueID == -1)
                return false;

            // Check if the queue is empty
            if (queue.empty())
            {
                queue.push_back(newPacket);
                return true;
            }

            // Iterate through the queue
            for (size_t i = 0; i < queue.size(); i++)
            {
                // Try to get the value ID of the current packet
                auto currentPacketValueID = tryGetValueID(queue[i]);
                if (currentPacketValueID == -1)
                    continue;

                // Check if the value ID matches
                if (currentPacketValueID == valueID)
                {
                    // Replace the packet
                    queue[i] = newPacket;
                    return true;
                }
            }

            // Add the packet to the queue
            queue.push_back(newPacket);
            return true;
        }

    private:
        /**
         * Tries to retrieve the value ID from a packet.
         * This is necessary since C++ does not support dynamic casting of templated types.
         * @param serialPacket The packet to retrieve the NTID from.
         * @return The NTID if it was found, or -1 if it was not found.
         */
        static int32_t tryGetValueID(std::shared_ptr<SerialPacket> serialPacket)
        {
            if (auto updateBoolPacket = std::dynamic_pointer_cast<UpdateBoolPacket>(serialPacket))
                return updateBoolPacket->valueID;
            if (auto updateIntPacket = std::dynamic_pointer_cast<UpdateIntPacket>(serialPacket))
                return updateIntPacket->valueID;
            if (auto updateFloatPacket = std::dynamic_pointer_cast<UpdateFloatPacket>(serialPacket))
                return updateFloatPacket->valueID;
            if (auto updateDoublePacket = std::dynamic_pointer_cast<UpdateDoublePacket>(serialPacket))
                return updateDoublePacket->valueID;
            if (auto updateStringPacket = std::dynamic_pointer_cast<UpdateStringPacket>(serialPacket))
                return updateStringPacket->valueID;
            if (auto updateBoolArrayPacket = std::dynamic_pointer_cast<UpdateBoolArrayPacket>(serialPacket))
                return updateBoolArrayPacket->valueID;
            if (auto updateIntArrayPacket = std::dynamic_pointer_cast<UpdateIntArrayPacket>(serialPacket))
                return updateIntArrayPacket->valueID;
            if (auto updateFloatArrayPacket = std::dynamic_pointer_cast<UpdateFloatArrayPacket>(serialPacket))
                return updateFloatArrayPacket->valueID;
            if (auto updateDoubleArrayPacket = std::dynamic_pointer_cast<UpdateDoubleArrayPacket>(serialPacket))
                return updateDoubleArrayPacket->valueID;
            return -1;
        }
    };
}