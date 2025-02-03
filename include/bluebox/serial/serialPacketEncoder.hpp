#pragma once

#include <cstdint>
#include "types/serialPacket.hpp"
#include "types/encodedSerialPacket.hpp"
#include "../utils/checksum.hpp"

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
            // Serialize the packet
            EncodedSerialPacket *encodedPacket = SerialPacketTypes::serialize(packet);

            // Create the packet buffer
            static uint8_t *packetBuffer = new uint8_t[MAX_BUFFER_SIZE];
            BufferWriter packerWriter(packetBuffer, MAX_BUFFER_SIZE);

            // Header
            packerWriter.writeUInt8(0xC9);
            packerWriter.writeUInt8(0x36);
            packerWriter.writeUInt8(0xB8);
            packerWriter.writeUInt8(0x47);

            // Packet
            packerWriter.writeUInt8((uint8_t)encodedPacket->type);                       // Type
            packerWriter.writeUInt8(encodedPacket->id);                                  // ID
            packerWriter.writeUInt16LE(encodedPacket->payloadSize);                      // Payload Size
            packerWriter.writeBytes(encodedPacket->payload, encodedPacket->payloadSize); // Payload

            // Checksum
            uint16_t checksum = Checksum::calc(buffer, packerWriter.getOffset() - 2);
            packerWriter.writeUInt16LE(checksum);

            return packerWriter.getOffset();
        }

    private:
        static constexpr size_t MAX_BUFFER_SIZE = 256;
    };
}