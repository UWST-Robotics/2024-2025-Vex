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
    typedef vexbridge::UpdateValuePacket<std::string> UpdateStringPacket;

    struct UpdateStringPacketType : public UpdateValuePacketType<std::string>
    {
        UpdateStringPacketType() : UpdateValuePacketType(SerialPacketTypeID::UPDATE_STRING)
        {
        }

        std::string deserializeValue(BufferReader &reader) override
        {
            return reader.readString8();
        }

        void serializeValue(BufferWriter &writer, std::string value) override
        {
            writer.writeString8(value);
        }
    };
}