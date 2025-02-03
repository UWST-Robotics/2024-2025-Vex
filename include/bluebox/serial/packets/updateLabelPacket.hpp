#pragma once

#include <cstdint>
#include <string>
#include "../serialPacket.hpp"
#include "../packetType.hpp"

namespace bluebox
{
    struct UpdateLabelPacket : public SerialPacket
    {
        UpdateLabelPacket(
            uint16_t ntID,
            std::string label)
            : ntID(ntID),
              label(label),
              SerialPacket(PacketType::UPDATE_LABEL)
        {
        }

        size_t serialize(uint8_t *buffer) override
        {
            // Serialize params
            buffer[0] = ntID >> 8;
            buffer[1] = ntID & 0xFF;

            buffer[2] = label.length() >> 8;
            buffer[3] = label.length() & 0xFF;

            for (int i = 0; i < label.length(); i++)
                buffer[i + 4] = label[i];

            return 4 + label.length();
        }

        static UpdateLabelPacket *deserialize(uint8_t *payload, uint16_t length)
        {
            // Check if the payload is too small
            if (length < 4)
                return nullptr;

            // Extract label params
            uint16_t ntID = (payload[0] << 8) | payload[1];
            uint16_t labelLength = (payload[2] << 8) | payload[3];

            // Check if the payload is too small
            if (length < 4 + labelLength)
                return nullptr;

            // Extract label
            std::string label = std::string((char *)&payload[3], labelLength);

            // Create packet
            return new UpdateLabelPacket(ntID, label);
        }

    private:
        uint16_t ntID;
        std::string label;
    };
}