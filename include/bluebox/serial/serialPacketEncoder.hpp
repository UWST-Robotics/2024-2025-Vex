#pragma once

#include <cstdint>
#include "serialPacket.hpp"
#include "serialChecksum.hpp"

namespace bluebox
{
    /**
     * Encodes serial packets for transmission over a serial port.
     */
    class SerialPacketEncoder
    {
    public:
        /**
         * Encodes a serializable packet into a complete packet.
         * @param packet The packet to encode.
         * @param buffer The buffer to write the encoded packet to.
         * @return The size of the encoded packet.
         */
        static size_t encode(SerialPacket *packet, uint8_t *buffer)
        {
            // Serialize the payload
            static uint8_t *payloadBuffer = new uint8_t[MAX_BUFFER_SIZE];
            size_t payloadSize = packet->serialize(payloadBuffer);

            // Calculate the size of the payload
            uint16_t packetSize = HEADER_SIZE +
                                  PACKET_TYPE_SIZE +
                                  PACKET_ID_SIZE +
                                  PACKET_LENGTH_SIZE +
                                  payloadSize +
                                  CHECKSUM_SIZE;

            // Header
            buffer[0] = 0xC9;
            buffer[1] = 0x36;
            buffer[2] = 0xB8;
            buffer[3] = 0x47;

            // Type
            buffer[4] = (uint8_t)packet->getPacketType();

            // ID
            buffer[5] = packet->getPacketID();

            // Payload Size
            buffer[6] = payloadSize >> 8;
            buffer[7] = payloadSize & 0xFF;

            // Payload
            uint8_t payloadOffset = HEADER_SIZE + PACKET_TYPE_SIZE + PACKET_ID_SIZE + PACKET_LENGTH_SIZE;
            for (uint16_t i = 0; i < payloadSize; i++)
                buffer[payloadOffset + i] = payloadBuffer[i];

            // Checksum
            uint16_t checksum = SerialChecksum::calc(buffer, packetSize - CHECKSUM_SIZE);
            buffer[packetSize - 2] = checksum >> 8;
            buffer[packetSize - 1] = checksum & 0xFF;

            return packetSize;
        }

    private:
        static constexpr uint16_t HEADER_SIZE = 4;
        static constexpr uint16_t PACKET_TYPE_SIZE = 1;
        static constexpr uint16_t PACKET_ID_SIZE = 1;
        static constexpr uint16_t PACKET_LENGTH_SIZE = 2;
        static constexpr uint16_t CHECKSUM_SIZE = 2;
        static constexpr size_t MAX_BUFFER_SIZE = 256;
    };
}