#pragma once

#include <deque>
#include "types/serialPacket.hpp"
#include "packets/updateValuePacket.hpp"
#include "types/serialPacketTypeID.hpp"
#include "packets/batchPacket.hpp"

namespace vexbridge
{
    class SerialQueue
    {
    public:
        /**
         * Adds a serial packet to the write queue.
         * @param packet The packet to write.
         */
        void push(SerialPacket *packet)
        {
            // Check if the queue is full
            if (queue.size() >= MAX_QUEUE_SIZE)
            {
                delete packet;
                return;
            }

            // Remove all value packets with the same ntID
            int32_t ntID = tryGetNTIDOfPacket(packet);
            if (ntID >= 0)
                removeAllValuePacketsOfID(ntID);

            // Push the packet to the queue
            queue.push_back(packet);
        }

        /**
         * Pops a packet from the queue.
         * @return The packet popped from the queue or nullptr if the queue is empty.
         */
        SerialPacket *pop()
        {
            // Check if the queue is empty
            if (queue.empty())
                return nullptr;

            // Try to batch packets
            // TODO: Fix me
            // BatchPacket *batchPacket = tryBatch();
            // if (batchPacket != nullptr)
            //     return batchPacket;

            // Pop the next packet from the queue
            SerialPacket *packet = queue.front();
            queue.pop_front();
            return packet;
        }

        /**
         * Checks if the queue is empty.
         * @return True if the queue is empty, false otherwise.
         */
        bool empty()
        {
            return queue.empty();
        }

    private:
        /**
         * Tries to get the NT ID of a packet.
         * @param packet The packet to get the NT ID of.
         * @return The NT ID of the packet or -1 if not found.
         */
        int32_t tryGetNTIDOfPacket(SerialPacket *packet)
        {
            int32_t ntID = -1;
            tryGetNTIDOfPacketType<bool>(packet, &ntID);
            tryGetNTIDOfPacketType<int>(packet, &ntID);
            tryGetNTIDOfPacketType<float>(packet, &ntID);
            tryGetNTIDOfPacketType<double>(packet, &ntID);
            tryGetNTIDOfPacketType<std::string>(packet, &ntID);
            return ntID;
        }
        template <typename T>
        void tryGetNTIDOfPacketType(SerialPacket *packet, int32_t *ntID)
        {
            auto *ntPacket = dynamic_cast<UpdateValuePacket<T> *>(packet);
            if (ntPacket == nullptr)
                return;
            *ntID = (int32_t)ntPacket->ntID;
        }

        /**
         * Removes all value packets with the specified ntID and type from the queue.
         * @param ntID The ntID to remove.
         * @param type The type to remove.
         */
        void removeAllValuePacketsOfID(uint16_t ntID)
        {
            // Loop through the queue
            for (size_t i = 0; i < queue.size(); i++)
            {
                // Try to get the NT ID of the packet
                uint16_t packetNTID = tryGetNTIDOfPacket(queue[i]);
                if (packetNTID != ntID)
                    continue;

                // Delete the packet
                delete queue[i];
                queue.erase(queue.begin() + i);
                i--;
            }
        }

        /**
         * Tries to batch front packets with others in the queue of the same type.
         * @return The batch packet or nullptr if no batch was created.
         */
        BatchPacket *tryBatch()
        {
            // Check if the queue is empty
            if (queue.empty())
                return nullptr;

            // Get the first packet in the queue
            SerialPacket *firstPacket = queue.front();

            // Check if there are any other packets of the same type
            bool hasOtherPacketsOfType = false;
            for (size_t i = 1; i < queue.size(); i++)
            {
                if (queue[i]->type == firstPacket->type)
                {
                    hasOtherPacketsOfType = true;
                    break;
                }
            }

            // Return if there are no other packets of the same type
            if (!hasOtherPacketsOfType)
                return nullptr;

            // Create a new batch packet
            BatchPacket *batchPacket = new BatchPacket();
            batchPacket->type = SerialPacketTypeID::BATCH_PACKET;
            batchPacket->subType = firstPacket->type;

            // Loop through the queue
            for (size_t i = 0; i < queue.size(); i++)
            {
                // Check if the packet is the same type as the first packet
                if (queue[i]->type != firstPacket->type)
                    continue;

                // Add the packet to the batch
                batchPacket->subPackets.push_back(queue[i]);

                // Remove the packet from the queue
                queue.erase(queue.begin() + i);
                i--;
            }

            // Return the batch packet
            return batchPacket;
        }

        static constexpr size_t MAX_QUEUE_SIZE = 512;

        std::deque<SerialPacket *> queue;
    };
}