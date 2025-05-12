#pragma once

#include <cstdint>
#include <cstring>
#include "common/serialPacket.h"
#include "common/serialPacketType.h"
#include "common/encodedSerialPacket.h"
#include "../../utils/bufferWriter.hpp"
#include "../../utils/bufferReader.hpp"

using namespace vexbridge::utils;

namespace vexbridge::serial
{
    template <typename T>
    struct UpdateValuePacket : public SerialPacket
    {
        uint16_t valueID;
        T newValue;
    };

    template <typename T>
    struct UpdateValuePacketType : public SerialPacketType
    {
        UpdateValuePacketType(SerialPacketTypeID typeID) : SerialPacketType(typeID)
        {
        }

        virtual T deserializeValue(BufferReader &reader) = 0;
        virtual void serializeValue(BufferWriter &writer, T value) = 0;

        std::unique_ptr<SerialPacket> deserialize(const EncodedSerialPacket &packet) override
        {
            // Make new update value packet
            auto newPacket = std::make_unique<UpdateValuePacket<T>>();
            newPacket->id = packet.id;
            newPacket->type = packet.type;

            // Read packet contents from payload
            BufferReader reader(packet.payload);
            newPacket->valueID = reader.readUInt16BE();
            newPacket->newValue = deserializeValue(reader);
            return newPacket;
        }

        std::unique_ptr<EncodedSerialPacket> serialize(const SerialPacket &packet) override
        {
            // Cast packet to update value packet
            const UpdateValuePacket<T> &updateValuePacket = dynamic_cast<const UpdateValuePacket<T> &>(packet);

            // Allocate buffer for payload
            Buffer payload;
            BufferWriter writer(payload);

            // Write value
            writer.writeUInt16BE(updateValuePacket.valueID);
            serializeValue(writer, updateValuePacket.newValue);

            // Make new encoded packet
            auto newPacket = std::make_unique<EncodedSerialPacket>();
            newPacket->type = packet.type;
            newPacket->id = packet.id;
            newPacket->payload = payload;
            return newPacket;
        }
    };
}