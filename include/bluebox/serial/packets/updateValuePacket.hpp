#pragma once

#include <cstdint>
#include "../serialPacket.hpp"
#include "../packetType.hpp"
#include <cstring>
#include "../../utils/ntLogger.hpp"

namespace bluebox
{
    struct UpdateValuePacket : public SerialPacket
    {
        enum class ValueType : uint8_t
        {
            BOOLEAN = 0x01,
            INT = 0x02,
            DOUBLE = 0x03
        };

        UpdateValuePacket(
            uint16_t ntID,
            uint16_t timestamp,
            ValueType valueType,
            void *newValue)
            : ntID(ntID),
              timestamp(timestamp),
              valueType(valueType),
              newValue(newValue),
              SerialPacket(PacketType::UPDATE_VALUE)
        {
        }

        size_t serialize(uint8_t *buffer) override
        {
            buffer[0] = ntID >> 8;
            buffer[1] = ntID & 0xFF;

            buffer[2] = timestamp >> 8;
            buffer[3] = timestamp & 0xFF;

            buffer[4] = (uint8_t)valueType;

            switch (valueType)
            {
            case ValueType::BOOLEAN:
                buffer[5] = *(bool *)newValue;
                break;
            case ValueType::INT:
                buffer[5] = *(int16_t *)newValue >> 8;
                buffer[6] = *(int16_t *)newValue & 0xFF;
                break;
            case ValueType::DOUBLE:
                memcpy(&buffer[5], newValue, 8);
                break;
            }

            return 5 + getSize(valueType);
        }

        static UpdateValuePacket *deserialize(uint8_t *payload, uint16_t length)
        {
            if (length < 5)
                return nullptr;

            uint16_t ntID = (payload[0] << 8) | payload[1];
            uint16_t timestamp = (payload[2] << 8) | payload[3];
            ValueType valueType = (ValueType)payload[4];

            if (length < 5 + UpdateValuePacket::getSize(valueType))
                return nullptr;

            switch (valueType)
            {
            case ValueType::BOOLEAN:
                return new UpdateValuePacket(ntID, timestamp, valueType, new bool(payload[5]));
            case ValueType::INT:
                return new UpdateValuePacket(ntID, timestamp, valueType, new int16_t((payload[5] << 8) | payload[6]));
            case ValueType::DOUBLE:
                return new UpdateValuePacket(ntID, timestamp, valueType, new double(*(double *)&payload[5]));
            }

            // Unknown value type
            NTLogger::logWarning("Unknown value type " + std::to_string((uint8_t)valueType));

            return nullptr;
        }

    private:
        /**
         * Gets the size in bytes of a value type.
         * @param valueType The value type.
         * @return The size in bytes of the value type.
         */
        static uint16_t getSize(ValueType valueType)
        {
            switch (valueType)
            {
            case ValueType::BOOLEAN:
                return 1;
            case ValueType::INT:
                return 2;
            case ValueType::DOUBLE:
                return 8;
            }
        }

        uint16_t ntID;
        uint16_t timestamp;
        ValueType valueType;
        void *newValue;
    };
}