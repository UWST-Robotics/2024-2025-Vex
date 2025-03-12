#pragma once

#include <cstdint>
#include <cstring>
#include "../types/serialPacket.hpp"
#include "../types/serialPacketType.hpp"
#include "../../utils/bufferWriter.hpp"
#include "../../utils/bufferReader.hpp"
#include "updateValuePacket.hpp"

namespace vexbridge::serial
{
    typedef vexbridge::serial::UpdateValuePacket<int> UpdateIntPacket;

    struct UpdateIntPacketType : public UpdateValuePacketType<int>
    {
        UpdateIntPacketType() : UpdateValuePacketType(SerialPacketTypeID::UPDATE_INT)
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