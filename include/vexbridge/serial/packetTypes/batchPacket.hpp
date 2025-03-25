#pragma once

#include <cstdint>
#include <string>
#include "common/serialPacket.h"
#include "common/serialPacketType.h"
#include "common/encodedSerialPacket.h"
#include "../../utils/bufferWriter.hpp"
#include "../../utils/bufferReader.hpp"
#include "../serialization/serialPacketDecoder.hpp"

using namespace vexbridge::utils;

namespace vexbridge::serial
{
    struct BatchPacket : public SerialPacket
    {
        SerialPacketTypeID subType;
        std::vector<std::unique_ptr<SerialPacket>> subPackets;
    };

    struct BatchPacketType : public SerialPacketType
    {
        BatchPacketType() : SerialPacketType(SerialPacketTypeID::BATCH_PACKET)
        {
        }

        std::unique_ptr<SerialPacket> deserialize(const EncodedSerialPacket &packet) override
        {
            // Make new batch packet
            auto newPacket = std::make_unique<BatchPacket>();
            newPacket->type = packet.type;
            newPacket->id = packet.id;

            // Read packet contents from payload
            BufferReader reader(packet.payload);
            newPacket->subType = (SerialPacketTypeID)reader.readUInt8();

            // Find the packet type
            SerialPacketType *packetType = AllPacketTypes::get(newPacket->subType);
            if (packetType == nullptr)
                throw std::runtime_error("Unknown packet type " + std::to_string((int)newPacket->subType));

            // Create Temporary Packet to store payload
            EncodedSerialPacket tempPacket;
            tempPacket.id = newPacket->id;
            tempPacket.type = newPacket->subType;

            // Read sub packets
            while (reader.getBytesAvailable() > 0)
            {
                // Read the length of the sub packet
                uint16_t subPacketLength = reader.readUInt8();

                // Read the sub packet payload
                tempPacket.payload = reader.readBytes(subPacketLength);

                // Split the payload into buffer for the sub packet
                auto subPacket = packetType->deserialize(tempPacket);
                newPacket->subPackets.push_back(std::move(subPacket));
            }

            return newPacket;
        }

        std::unique_ptr<EncodedSerialPacket> serialize(const SerialPacket &packet) override
        {
            // Cast packet to batch packet
            const BatchPacket &batchPacket = dynamic_cast<const BatchPacket &>(packet);

            // Allocate buffer for payload
            Buffer payload;
            BufferWriter writer(payload);

            // Write value
            writer.writeUInt8((uint8_t)batchPacket.subType);

            // Find the packet type
            SerialPacketType *packetType = AllPacketTypes::get(batchPacket.subType);
            if (packetType == nullptr)
                throw std::runtime_error("Unknown packet type " + std::to_string((int)batchPacket.subType));

            // Serialize sub packets
            for (const auto &subPacket : batchPacket.subPackets)
            {
                // Serialize the sub packet
                auto encodedSubPacket = packetType->serialize(*subPacket.get());

                // Write the length of the sub packet
                writer.writeUInt8(encodedSubPacket->payload.size());

                // Write the sub packet payload
                writer.writeBytes(encodedSubPacket->payload, encodedSubPacket->payload.size());
            }

            // Make new encoded packet
            auto newPacket = std::make_unique<EncodedSerialPacket>();
            newPacket->type = packet.type;
            newPacket->id = packet.id;
            newPacket->payload = payload;
            return newPacket;
        }
    };
}