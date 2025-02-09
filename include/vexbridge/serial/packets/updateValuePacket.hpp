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
    struct UpdateValuePacket : public SerialPacket
    {
        enum class ValueType : uint8_t
        {
            BOOLEAN = 0x01,
            INT = 0x02,
            FLOAT = 0x03,
            DOUBLE = 0x04
        };

        uint16_t ntID = 0;
        ValueType valueType = ValueType::BOOLEAN;
        void *newValue = nullptr;

        UpdateValuePacket()
        {
        }
        UpdateValuePacket(const UpdateValuePacket &other)
        {
            ntID = other.ntID;
            valueType = other.valueType;

            // Copy the value
            switch (valueType)
            {
            case ValueType::BOOLEAN:
                newValue = new bool(*(bool *)other.newValue);
                break;
            case ValueType::INT:
                newValue = new int16_t(*(int16_t *)other.newValue);
                break;
            case ValueType::FLOAT:
                newValue = new float(*(float *)other.newValue);
                break;
            case ValueType::DOUBLE:
                newValue = new double(*(double *)other.newValue);
                break;
            }
        }
        ~UpdateValuePacket()
        {
            if (newValue != nullptr)
                delete newValue;
        }
    };

    struct UpdateValuePacketType : public SerialPacketType
    {
        UpdateValuePacketType()
            : SerialPacketType(SerialPacketTypeID::UPDATE_VALUE)
        {
        }

        SerialPacket *deserialize(EncodedSerialPacket *packet) override
        {
            BufferReader reader(packet->payload, packet->payloadSize);
            UpdateValuePacket *newPacket = new UpdateValuePacket();
            newPacket->type = packet->type;
            newPacket->id = packet->id;
            newPacket->ntID = reader.readUInt16BE();
            newPacket->valueType = (UpdateValuePacket::ValueType)reader.readUInt8();

            switch (newPacket->valueType)
            {
            case UpdateValuePacket::ValueType::BOOLEAN:
                newPacket->newValue = new bool(reader.readUInt8());
                break;
            case UpdateValuePacket::ValueType::INT:
                newPacket->newValue = new int16_t(reader.readUInt16BE());
                break;
            case UpdateValuePacket::ValueType::FLOAT:
                newPacket->newValue = new float(reader.readFloatBE());
                break;
            case UpdateValuePacket::ValueType::DOUBLE:
                newPacket->newValue = new double(reader.readDoubleBE());
                break;
            }

            return newPacket;
        }

        EncodedSerialPacket *serialize(SerialPacket *packet) override
        {
            UpdateValuePacket *updateValuePacket = dynamic_cast<UpdateValuePacket *>(packet);
            if (updateValuePacket == nullptr)
                return nullptr;

            size_t payloadSize = 3 + 8;
            uint8_t *payload = new uint8_t[payloadSize];
            BufferWriter writer(payload, payloadSize);

            writer.writeUInt16BE(updateValuePacket->ntID);
            writer.writeUInt8((uint8_t)updateValuePacket->valueType);

            switch (updateValuePacket->valueType)
            {
            case UpdateValuePacket::ValueType::BOOLEAN:
                writer.writeUInt8(*(bool *)updateValuePacket->newValue);
                break;
            case UpdateValuePacket::ValueType::INT:
                writer.writeUInt16BE(*(int16_t *)updateValuePacket->newValue);
                break;
            case UpdateValuePacket::ValueType::FLOAT:
                writer.writeFloatBE(*(float *)updateValuePacket->newValue);
                break;
            case UpdateValuePacket::ValueType::DOUBLE:
                writer.writeDoubleBE(*(double *)updateValuePacket->newValue);
                break;
            }

            return EncodedSerialPacket::build(packet, payload, writer.getOffset());
        }
    };
}