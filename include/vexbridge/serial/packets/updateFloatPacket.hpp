#pragma once

#include <cstdint>
#include <cstring>
#include "../types/serialPacket.hpp"
#include "../types/serialPacketType.hpp"
#include "../../utils/ntLogger.hpp"
#include "../../utils/bufferWriter.hpp"
#include "../../utils/bufferReader.hpp"
#include "updateValuePacket.hpp"

namespace vexbridge
{
    typedef vexbridge::UpdateValuePacket<float> UpdateFloatPacket;

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