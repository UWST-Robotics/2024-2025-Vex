#pragma once
#include <cstdint>
#include <memory>
#include "serialPacket.h"

namespace vexbridge::serial
{
    /**
     * A packet that was already sent to the VEXBridge and is waiting for a response.
     */
    struct SentSerialPacket
    {
        /// @brief The packet that was sent
        std::shared_ptr<SerialPacket> packet;

        /// @brief Timestamp of when the packet was sent
        uint32_t timestamp;

        /// @brief Number of retries sent for this packet
        uint8_t retries = 0;
    };
}