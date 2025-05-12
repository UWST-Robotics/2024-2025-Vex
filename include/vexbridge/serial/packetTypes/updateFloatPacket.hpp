#pragma once

#include <cstdint>
#include <cstring>
#include "common/serialPacket.h"
#include "common/serialPacketType.h"
#include "common/encodedSerialPacket.h"
#include "../../utils/bufferWriter.hpp"
#include "../../utils/bufferReader.hpp"
#include "updateValuePacket.hpp"

namespace vexbridge::serial
{
    typedef vexbridge::serial::UpdateValuePacket<float> UpdateFloatPacket;

    struct UpdateFloatPacketType : public UpdateValuePacketType<float>
    {
        UpdateFloatPacketType() : UpdateValuePacketType(SerialPacketTypeID::UPDATE_FLOAT)
        {
        }

        float deserializeValue(BufferReader &reader) override
        {
            return reader.readFloatBE();
        }

        void serializeValue(BufferWriter &writer, float value) override
        {
            writer.writeFloatBE(value);
        }
    };
}