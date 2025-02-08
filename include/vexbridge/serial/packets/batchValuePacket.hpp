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
        uint16_t timestamp = 0;
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
            BatchValuePacket *batchPacket = new BatchValuePacket();
            batchPacket->type = SerialPacketTypeID::BATCH_VALUE;
            batchPacket->id = 0;

            while (!queue.empty() && batchPacket->subPackets.size() < MAX_SUBPACKETS)
            {
                // Get the next packet
                SerialPacket *packet = queue.front();

                // Check for UpdateValuePacket
                UpdateValuePacket *valuePacket = dynamic_cast<UpdateValuePacket *>(packet);
                if (valuePacket == nullptr)
                    break;

                // Remove the packet from the queue
                queue.pop();

                // Append the packet to the batch
                batchPacket->subPackets.push_back(valuePacket);
                batchPacket->timestamp = valuePacket->timestamp;
            }

            // Check if any packets were added
            if (batchPacket->subPackets.size() > 0)
                queue.push(batchPacket);
            else
                delete batchPacket;
        }

        static constexpr uint8_t MAX_SUBPACKETS = 30;
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
            newPacket->timestamp = reader.readUInt16BE();

            uint8_t subPacketCount = reader.readUInt8();
            newPacket->subPackets.reserve(subPacketCount);

            // Iterate through each sub-packet
            for (uint8_t i = 0; i < subPacketCount; i++)
            {
                // Copy the parent packet data
                UpdateValuePacket *subPacket = new UpdateValuePacket();
                subPacket->id = packet->id;
                subPacket->type = SerialPacketTypeID::UPDATE_VALUE;
                subPacket->timestamp = newPacket->timestamp;

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
            size_t payloadSize = 3;
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
                case UpdateValuePacket::ValueType::DOUBLE:
                    payloadSize += 8;
                    break;
                }
            }

            // Allocate the payload buffer
            uint8_t *payload = new uint8_t[payloadSize];
            BufferWriter writer(payload, payloadSize);

            // Write the packet data
            writer.writeUInt16BE(batchValuePacket->timestamp);
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
                case UpdateValuePacket::ValueType::DOUBLE:
                    writer.writeDoubleBE(*(double *)subPacket->newValue);
                    break;
                }
            }

            return EncodedSerialPacket::build(packet, payload, writer.getOffset());
        }
    };
}