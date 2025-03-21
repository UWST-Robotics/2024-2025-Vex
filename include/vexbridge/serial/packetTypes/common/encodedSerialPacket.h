#pragma once

#include <cstdint>
#include <cstddef>
#include "serialPacket.h"
#include "../utils/buffer.h"

using namespace vexbridge::utils;

namespace vexbridge::serial
{
    /**
     * Represents a packet that includes a serialized payload.
     */
    struct EncodedSerialPacket : SerialPacket
    {
        Buffer payload;
    };
}