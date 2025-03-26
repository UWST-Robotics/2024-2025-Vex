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
    typedef vexbridge::serial::UpdateValuePacket<std::string> UpdateStringPacket;

    struct UpdateStringPacketType : public UpdateValuePacketType<std::string>
    {
        UpdateStringPacketType() : UpdateValuePacketType(SerialPacketTypeID::UPDATE_STRING)
        {
        }

        std::string deserializeValue(BufferReader &reader) override
        {
            return reader.readString16();
        }

        void serializeValue(BufferWriter &writer, std::string value) override
        {
            writer.writeString16(value);
        }
    };
}