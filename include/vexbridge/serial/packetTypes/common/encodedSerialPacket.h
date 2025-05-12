#pragma once

#include <cstdint>
#include <cstddef>
#include "serialPacket.h"
#include "../../../utils/buffer.h"

using namespace vexbridge::utils;

namespace vexbridge::serial
{
    /**
     * Represents a packet that includes a serialized payload.
     */
    struct EncodedSerialPacket : SerialPacket
    {
        /// @brief Raw buffer containing the specific packet data unique to the packet type (the payload)
        Buffer payload;
    };
}