#pragma once

#include <cstdint>
#include "../serialPacket.hpp"
#include "../packetType.hpp"

namespace bluebox
{
    struct GenericAckPacket : public SerialPacket
    {
        GenericAckPacket() : SerialPacket(PacketType::GENERIC_ACK)
        {
        }

        size_t serialize(uint8_t *buffer) override
        {
            return 0;
        }

        static GenericAckPacket *deserialize(uint8_t *payload, uint16_t length)
        {
            return new GenericAckPacket();
        }
    };
}