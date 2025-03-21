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
    typedef vexbridge::serial::UpdateValuePacket<double> UpdateDoublePacket;

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