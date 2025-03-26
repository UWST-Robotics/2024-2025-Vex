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
    typedef vexbridge::serial::UpdateValueArrayPacket<bool> UpdateBoolArrayPacket;

    struct UpdateBoolArrayPacketType : public UpdateValueArrayPacketType<bool>
    {
        UpdateBoolArrayPacketType() : UpdateValueArrayPacketType(SerialPacketTypeID::UPDATE_BOOL_ARRAY)
        {
        }

        bool deserializeValue(BufferReader &reader) override
        {
            return reader.readUInt8() != 0;
        }

        void serializeValue(BufferWriter &writer, bool value) override
        {
            writer.writeUInt8(value ? 1 : 0);
        }
    };
}