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
    typedef vexbridge::UpdateValuePacket<double> UpdateDoublePacket;

    struct UpdateDoublePacketType : public UpdateValuePacketType<double>
    {
        UpdateDoublePacketType() : UpdateValuePacketType(SerialPacketTypeID::UPDATE_DOUBLE)
        {
        }

        double deserializeValue(BufferReader &reader) override
        {
            return reader.readDoubleBE();
        }

        void serializeValue(BufferWriter &writer, double value) override
        {
            writer.writeDoubleBE(value);
        }
    };
}