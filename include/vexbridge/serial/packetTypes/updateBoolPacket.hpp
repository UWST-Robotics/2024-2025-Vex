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
    typedef vexbridge::serial::UpdateValuePacket<bool> UpdateBoolPacket;

    struct UpdateBoolPacketType : public UpdateValuePacketType<bool>
    {
        UpdateBoolPacketType() : UpdateValuePacketType(SerialPacketTypeID::UPDATE_BOOL)
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