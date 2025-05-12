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
    typedef vexbridge::serial::UpdateValueArrayPacket<int> UpdateIntArrayPacket;

    struct UpdateIntArrayPacketType : public UpdateValueArrayPacketType<int>
    {
        UpdateIntArrayPacketType() : UpdateValueArrayPacketType(SerialPacketTypeID::UPDATE_INT_ARRAY)
        {
        }

        int deserializeValue(BufferReader &reader) override
        {
            return reader.readUInt16BE();
        }

        void serializeValue(BufferWriter &writer, int value) override
        {
            writer.writeUInt16BE(value);
        }
    };
}