#pragma once

#include <cstdint>
#include <cstring>
#include <vector>
#include <queue>
#include "../types/serialPacket.hpp"
#include "../types/serialPacketType.hpp"
#include "../../utils/ntLogger.hpp"
#include "../../utils/bufferWriter.hpp"
#include "../../utils/bufferReader.hpp"
#include "updateValuePacket.hpp"

namespace vexbridge
{
    struct BatchValuePacket : public SerialPacket
    {
        std::vector<UpdateValuePacket *> subPackets;

        ~BatchValuePacket()
        {
            for (UpdateValuePacket *subPacket : subPackets)
                delete subPacket;
        }

        /**
         * Tries to combine a queue of UpdateValuePackets into a single BatchValuePacket.
         * Automatically removes the packets from the queue and appends a new BatchValuePacket.
         * @param queue The queue of packets to combine.
         */
        static void tryCollapseQueue(std::queue<SerialPacket *> &queue)
        {
            // Create a new batch packet
            BatchValuePacket *newPacket = new BatchValuePacket();
            newPacket->type = SerialPacketTypeID::BATCH_VALUE;
            newPacket->id = 0;

            // Iterate through each packet in the queue
            while (!queue.empty())
            {
                // Check if the sub-packet limit has been reached
                if (newPacket->subPackets.size() >= MAX_SUBPACKETS)
                    break;

                // Get the next packet
                SerialPacket *packet = queue.front();

                // Check if the packet is a batch packet
                BatchValuePacket *batchPacket = dynamic_cast<BatchValuePacket *>(packet);
                if (batchPacket != nullptr)
                {
                    // Check if the batch packet is too large
                    if (newPacket->subPackets.size() + batchPacket->subPackets.size() > MAX_SUBPACKETS)
                        break;

                    // Remove the packet from the queue
                    queue.pop();

                    // Add each sub-packet to the batch
                    for (UpdateValuePacket *subPacket : batchPacket->subPackets)
                        newPacket->subPackets.push_back(new UpdateValuePacket(*subPacket));

                    // Delete the packet
                    delete batchPacket;
                    continue;
                }

                // Check if the packet is an update value packet
                UpdateValuePacket *valuePacket = dynamic_cast<UpdateValuePacket *>(packet);
                if (valuePacket != nullptr)
                {
                    // Remove the packet from the queue
                    queue.pop();

                    // Append the packet to the batch
                    newPacket->subPackets.push_back(new UpdateValuePacket(*valuePacket));

                    // Delete the packet
                    delete valuePacket;
                    continue;
                }

                // Otherwise, stop processing
                break;
            }

            // Check if any packets were added
            if (newPacket->subPackets.size() > 0)
                queue.push(newPacket);
            else
                delete newPacket;
        }

        static constexpr uint8_t MAX_SUBPACKETS = 20;
    };

    struct BatchValuePacketType : public SerialPacketType
    {
        BatchValuePacketType()
            : SerialPacketType(SerialPacketTypeID::BATCH_VALUE)
        {
        }

        SerialPacket *deserialize(EncodedSerialPacket *packet) override
        {
            BufferReader reader(packet->payload, packet->payloadSize);
            BatchValuePacket *newPacket = new BatchValuePacket();
            newPacket->type = packet->type;
            newPacket->id = packet->id;

            uint8_t subPacketCount = reader.readUInt8();
            newPacket->subPackets.reserve(subPacketCount);

            // Iterate through each sub-packet
            for (uint8_t i = 0; i < subPacketCount; i++)
            {
                // Copy the parent packet data
                UpdateValuePacket *subPacket = new UpdateValuePacket();
                subPacket->id = packet->id;
                subPacket->type = SerialPacketTypeID::UPDATE_VALUE;

                // Read the sub-packet data
                subPacket->ntID = reader.readUInt16BE();
                subPacket->valueType = (UpdateValuePacket::ValueType)reader.readUInt8();
                switch (subPacket->valueType)
                {
                case UpdateValuePacket::ValueType::BOOLEAN:
                    subPacket->newValue = new bool(reader.readUInt8());
                    break;
                case UpdateValuePacket::ValueType::INT:
                    subPacket->newValue = new int16_t(reader.readUInt16BE());
                    break;
                case UpdateValuePacket::ValueType::FLOAT:
                    subPacket->newValue = new float(reader.readFloatBE());
                    break;
                case UpdateValuePacket::ValueType::DOUBLE:
                    subPacket->newValue = new double(reader.readDoubleBE());
                    break;
                }

                // Add the sub-packet to the list
                newPacket->subPackets.push_back(subPacket);
            }
            return newPacket;
        }

        EncodedSerialPacket *serialize(SerialPacket *packet) override
        {
            BatchValuePacket *batchValuePacket = dynamic_cast<BatchValuePacket *>(packet);
            if (batchValuePacket == nullptr)
                return nullptr;

            // Calculate the payload size
            size_t payloadSize = 1;
            for (UpdateValuePacket *subPacket : batchValuePacket->subPackets)
            {
                payloadSize += 3;
                switch (subPacket->valueType)
                {
                case UpdateValuePacket::ValueType::BOOLEAN:
                    payloadSize += 1;
                    break;
                case UpdateValuePacket::ValueType::INT:
                    payloadSize += 2;
                    break;
                case UpdateValuePacket::ValueType::FLOAT:
                    payloadSize += 8;
                    break;
                case UpdateValuePacket::ValueType::DOUBLE:
                    payloadSize += 8;
                    break;
                }
            }

            // Allocate the payload buffer
            uint8_t *payload = new uint8_t[payloadSize];
            BufferWriter writer(payload, payloadSize);

            // Write the packet data
            writer.writeUInt8(batchValuePacket->subPackets.size());

            // Write each sub-packet
            for (UpdateValuePacket *subPacket : batchValuePacket->subPackets)
            {
                writer.writeUInt16BE(subPacket->ntID);
                writer.writeUInt8((uint8_t)subPacket->valueType);

                switch (subPacket->valueType)
                {
                case UpdateValuePacket::ValueType::BOOLEAN:
                    writer.writeUInt8(*(bool *)subPacket->newValue);
                    break;
                case UpdateValuePacket::ValueType::INT:
                    writer.writeUInt16BE(*(int16_t *)subPacket->newValue);
                    break;
                case UpdateValuePacket::ValueType::FLOAT:
                    writer.writeFloatBE(*(float *)subPacket->newValue);
                    break;
                case UpdateValuePacket::ValueType::DOUBLE:
                    writer.writeDoubleBE(*(double *)subPacket->newValue);
                    break;
                }
            }

            return EncodedSerialPacket::build(packet, payload, writer.getOffset());
        }
    };
}