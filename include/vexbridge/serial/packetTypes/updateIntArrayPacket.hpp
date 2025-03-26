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
    typedef vexbridge::serial::UpdateValueArrayPacket<int32_t> UpdateIntArrayPacket;

    struct UpdateIntArrayPacketType : public UpdateValueArrayPacketType<int32_t>
    {
        UpdateIntArrayPacketType() : UpdateValueArrayPacketType(SerialPacketTypeID::UPDATE_INT_ARRAY)
        {
        }

        int32_t deserializeValue(BufferReader &reader) override
        {
            return reader.readUInt16BE();
        }

        void serializeValue(BufferWriter &writer, int32_t value) override
        {
            writer.writeUInt16BE(value);
        }
    };
}