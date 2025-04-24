#pragma once

#include <cstdint>
#include <cstring>
#include "common/serialPacket.h"
#include "common/serialPacketType.h"
#include "common/encodedSerialPacket.h"
#include "../../utils/bufferWriter.hpp"
#include "../../utils/bufferReader.hpp"
#include <vector>

using namespace vexbridge::utils;

namespace vexbridge::serial
{
    template <typename T>
    using UpdateValueArrayPacket = vexbridge::serial::UpdateValuePacket<std::vector<T>>;

    template <typename T>
    struct UpdateValueArrayPacketType : public SerialPacketType
    {
        UpdateValueArrayPacketType(SerialPacketTypeID typeID) : SerialPacketType(typeID)
        {
        }

        virtual T deserializeValue(BufferReader &reader) = 0;
        virtual void serializeValue(BufferWriter &writer, T value) = 0;

        std::unique_ptr<SerialPacket> deserialize(const EncodedSerialPacket &packet) override
        {
            // Make new update value packet
            auto newPacket = std::make_unique<UpdateValueArrayPacket<T>>();
            newPacket->id = packet.id;
            newPacket->type = packet.type;

            // Read packet contents from payload
            BufferReader reader(packet.payload);
            newPacket->valueID = reader.readUInt16BE();
            uint16_t newValueSize = reader.readUInt16BE();
            for (int i = 0; i < newValueSize; i++)
                newPacket->newValue.push_back(deserializeValue(reader));
            return newPacket;
        }

        std::unique_ptr<EncodedSerialPacket> serialize(const SerialPacket &packet) override
        {
            // Cast packet to update value packet
            const UpdateValueArrayPacket<T> &updateValueArrayPacket = dynamic_cast<const UpdateValueArrayPacket<T> &>(packet);

            // Allocate buffer for payload
            Buffer payload;
            BufferWriter writer(payload);

            // Write value
            writer.writeUInt16BE(updateValueArrayPacket.valueID);
            writer.writeUInt16BE(updateValueArrayPacket.newValue.size());
            for (const T &value : updateValueArrayPacket.newValue)
                serializeValue(writer, value);

            // Make new encoded packet
            auto newPacket = std::make_unique<EncodedSerialPacket>();
            newPacket->type = packet.type;
            newPacket->id = packet.id;
            newPacket->payload = payload;
            return newPacket;
        }
    };
}