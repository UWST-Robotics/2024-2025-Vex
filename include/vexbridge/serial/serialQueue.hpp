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

            // Try to update an existing NT value in the queue
            if (updateNTValueInQueue(packet))
            {
                delete packet;
                return;
            }

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
         * Tries to find an existing NT value packet in the queue of a specific type and update its value.
         * @param packet The packet to check.
         * @return True if the packet was updated, false otherwise.
         */
        template <typename T>
        bool updateNTValueInQueueOfType(SerialPacket *packet)
        {
            // Case the packet to an update value packet
            auto *updatePacket = dynamic_cast<UpdateValuePacket<T> *>(packet);
            if (updatePacket == nullptr)
                return false;

            // Get the NT ID and new value
            int32_t ntID = updatePacket->ntID;
            T newValue = updatePacket->newValue;

            // Loop through the queue
            for (size_t i = 0; i < queue.size(); i++)
            {
                // Case the packet to an update value packet
                auto *queuePacket = dynamic_cast<UpdateValuePacket<T> *>(queue[i]);
                if (queuePacket == nullptr)
                    continue;

                // Check if the packet has the same NT ID
                if (queuePacket->ntID != ntID)
                    continue;

                // Update the value
                queuePacket->newValue = newValue;
                return true;
            }

            return false;
        }

        /**
         * Tries to find an existing NT value packet in the queue and update its value.
         * @param packet The packet to check.
         * @return True if the packet was updated, false otherwise.
         */
        bool updateNTValueInQueue(SerialPacket *packet)
        {
            return updateNTValueInQueueOfType<int>(packet) ||
                   updateNTValueInQueueOfType<float>(packet) ||
                   updateNTValueInQueueOfType<double>(packet) ||
                   updateNTValueInQueueOfType<bool>(packet) ||
                   updateNTValueInQueueOfType<std::string>(packet);
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