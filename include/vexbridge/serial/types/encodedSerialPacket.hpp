#pragma once

#include <cstdint>
#include <cstddef>
#include "serialPacket.hpp"

namespace vexbridge
{
    /**
     * Represents a packet that includes a serialized payload.
     */
    struct EncodedSerialPacket : SerialPacket
    {
        virtual ~EncodedSerialPacket()
        {
            delete[] payload;
        }

        static EncodedSerialPacket *build(SerialPacket *packet, uint8_t *payload, size_t payloadSize)
        {
            EncodedSerialPacket *encodedPacket = new EncodedSerialPacket();
            encodedPacket->type = packet->type;
            encodedPacket->id = packet->id;
            encodedPacket->payload = payload;
            encodedPacket->payloadSize = payloadSize;
            return encodedPacket;
        }

        uint8_t *payload = nullptr;
        size_t payloadSize = 0;
    };
}