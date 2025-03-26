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
    typedef vexbridge::serial::UpdateValueArrayPacket<double> UpdateDoubleArrayPacket;

    struct UpdateDoubleArrayPacketType : public UpdateValueArrayPacketType<double>
    {
        UpdateDoubleArrayPacketType() : UpdateValueArrayPacketType(SerialPacketTypeID::UPDATE_DOUBLE_ARRAY)
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