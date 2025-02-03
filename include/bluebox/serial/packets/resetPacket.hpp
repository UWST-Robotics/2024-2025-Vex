#pragma once

#include <cstdint>
#include "../serialPacket.hpp"
#include "../packetType.hpp"

namespace bluebox
{
    struct ResetPacket : public SerialPacket
    {
        ResetPacket() : SerialPacket(PacketType::RESET)
        {
        }

        size_t serialize(uint8_t *buffer) override
        {
            return 0;
        }
    };
}