#pragma once

#include <cstdint>
#include <cstring>
#include "../types/serialPacket.hpp"
#include "../types/serialPacketType.hpp"
#include "../../utils/ntLogger.hpp"
#include "../../utils/bufferWriter.hpp"
#include "../../utils/bufferReader.hpp"

namespace vexbridge
{
    template <typename T>
    struct UpdateValuePacket : public SerialPacket
    {
        uint16_t ntID = 0;
        T newValue = 0;
    };

    template <typename T>
    struct UpdateValuePacketType : public SerialPacketType
    {
        UpdateValuePacketType(SerialPacketTypeID typeID) : SerialPacketType(typeID)
        {
        }

        virtual T deserializeValue(BufferReader &reader) = 0;
        virtual void serializeValue(BufferWriter &writer, T value) = 0;

        SerialPacket *deserialize(EncodedSerialPacket *packet) override
        {
            BufferReader reader(packet->payload, packet->payloadSize);
            UpdateValuePacket<T> *newPacket = new UpdateValuePacket<T>();
            newPacket->type = packet->type;
            newPacket->id = packet->id;
            newPacket->ntID = reader.readUInt16BE();
            newPacket->newValue = deserializeValue(reader);
            return newPacket;
        }

        EncodedSerialPacket *serialize(SerialPacket *packet) override
        {
            UpdateValuePacket<T> *updateValuePacket = dynamic_cast<UpdateValuePacket<T> *>(packet);
            if (updateValuePacket == nullptr)
                return nullptr;

            size_t payloadSize = 2 + 255;
            static uint8_t *payload = new uint8_t[payloadSize];
            BufferWriter writer(payload, payloadSize);

            writer.writeUInt16BE(updateValuePacket->ntID);
            serializeValue(writer, updateValuePacket->newValue);

            return EncodedSerialPacket::build(packet, payload, writer.getOffset());
        }
    };
}