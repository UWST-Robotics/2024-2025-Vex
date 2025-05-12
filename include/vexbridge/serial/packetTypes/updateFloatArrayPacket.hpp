#pragma once

#include <cstdint>
#include <cstring>
#include "common/serialPacket.h"
#include "common/serialPacketType.h"
#include "common/encodedSerialPacket.h"
#include "../../utils/bufferWriter.hpp"
#include "../../utils/bufferReader.hpp"
#include "updateValueArrayPacket.hpp"

namespace vexbridge::serial
{
    typedef vexbridge::serial::UpdateValueArrayPacket<float> UpdateFloatArrayPacket;

    struct UpdateFloatArrayPacketType : public UpdateValueArrayPacketType<float>
    {
        UpdateFloatArrayPacketType() : UpdateValueArrayPacketType(SerialPacketTypeID::UPDATE_FLOAT_ARRAY)
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