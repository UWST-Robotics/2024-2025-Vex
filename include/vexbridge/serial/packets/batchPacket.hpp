#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include "../types/serialPacket.hpp"
#include "../types/serialPacketType.hpp"
#include "../../utils/bufferWriter.hpp"
#include "../../utils/bufferReader.hpp"
#include "../serialPacketTypes.hpp"

using namespace vexbridge::utils;

namespace vexbridge::serial
{
    struct BatchPacket : public SerialPacket
    {
        SerialPacketTypeID subType = SerialPacketTypeID::UNKNOWN;
        std::vector<SerialPacket *> subPackets;

        ~BatchPacket()
        {
            for (SerialPacket *packet : subPackets)
                delete packet;
        }
    };

    struct BatchPacketType : public SerialPacketType
    {
        static constexpr uint8_t MAX_SUBPACKETS = 20;

        BatchPacketType() : SerialPacketType(SerialPacketTypeID::BATCH_PACKET)
        {
        }

        SerialPacket *deserialize(EncodedSerialPacket *packet) override
        {
            BufferReader reader(packet->payload, packet->payloadSize);
            BatchPacket *newPacket = new BatchPacket();
            newPacket->type = packet->type;
            newPacket->id = packet->id;
            newPacket->subType = (SerialPacketTypeID)reader.readUInt8();

            // Get Subpacket Type
            SerialPacketType *subPacketType = SerialPacketTypes::get(newPacket->subType);
            if (subPacketType == nullptr)
            {
                delete newPacket;
                throw std::runtime_error("Invalid subpacket type.");
            }

            // Deserialize Subpackets
            uint8_t subPacketCount = 0;
            while (reader.hasData())
            {
                // Fallback in case of packet corruption
                if (subPacketCount >= MAX_SUBPACKETS)
                    break;
                subPacketCount++;

                // Build Encoded Subpacket from Payload
                EncodedSerialPacket subEncodedPacket;
                subEncodedPacket.type = newPacket->subType;
                subEncodedPacket.id = newPacket->id;
                subEncodedPacket.payloadSize = reader.readUInt8();
                subEncodedPacket.payload = reader.readBytes(subEncodedPacket.payloadSize);

                // Deserialize Subpacket
                SerialPacket *subPacket = subPacketType->deserialize(&subEncodedPacket);
                if (subPacket != nullptr)
                    newPacket->subPackets.push_back(subPacket);
            }

            return newPacket;
        }

        EncodedSerialPacket *serialize(SerialPacket *packet) override
        {
            BatchPacket *batchPacket = dynamic_cast<BatchPacket *>(packet);
            if (batchPacket == nullptr)
                return nullptr;

            // Verify sub-packets are all match this packet type
            for (SerialPacket *subPacket : batchPacket->subPackets)
                if (subPacket->type != batchPacket->subType)
                    throw std::runtime_error("Subpacket type mismatch.");

            // Verify more than 0 sub-packets
            if (batchPacket->subPackets.size() <= 0)
                throw std::runtime_error("No subpackets to batch.");

            // Find packet type
            SerialPacketType *subPacketType = SerialPacketTypes::get(batchPacket->subType);
            if (subPacketType == nullptr)
                throw std::runtime_error("Invalid subpacket type.");

            // Serialize all sub-packets
            size_t subPacketsPayloadLength = 1;
            std::vector<EncodedSerialPacket *> packetArr;
            for (SerialPacket *subPacket : batchPacket->subPackets)
            {
                // Serialize Subpacket
                EncodedSerialPacket *subEncodedPacket = subPacketType->serialize(subPacket);
                if (subEncodedPacket == nullptr)
                    throw std::runtime_error("Failed to serialize subpacket.");

                // Check payload size
                if (subEncodedPacket->payloadSize > 255)
                    throw std::runtime_error("Subpacket payload too large.");

                // Add to payload list
                subPacketsPayloadLength += 1 + subEncodedPacket->payloadSize;
                packetArr.push_back(subEncodedPacket);
            }

            // Verify all sub-packet payloads are less than 256 bytes
            for (EncodedSerialPacket *subEncodedPacket : packetArr)
                if (subEncodedPacket->payloadSize > 255)
                    throw std::runtime_error("Subpacket payload too large.");

            // Allocate buffer
            size_t payloadSize = 1 + subPacketsPayloadLength + packetArr.size();
            uint8_t *payload = new uint8_t[payloadSize];
            BufferWriter writer(payload, payloadSize);

            // Write Header
            writer.writeUInt8((uint8_t)batchPacket->subType);

            // Copy Subpackets
            for (EncodedSerialPacket *subEncodedPacket : packetArr)
            {
                writer.writeUInt8(subEncodedPacket->payloadSize);
                writer.writeBytes(subEncodedPacket->payload, subEncodedPacket->payloadSize);
            }

            return EncodedSerialPacket::build(packet, payload, writer.getOffset());
        }
    };
}