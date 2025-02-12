#pragma once

#include <cstdint>
#include "types/serialPacket.hpp"
#include "packets/updateValuePacket.hpp"
#include "packets/updateLabelPacket.hpp"
#include "packets/genericAckPacket.hpp"
#include "packets/resetPacket.hpp"
#include "../utils/bufferReader.hpp"
#include "../utils/checksum.hpp"
#include "serialPacketTypes.hpp"
#include "../utils/byteStuffer.hpp"

namespace vexbridge
{
    /**
     * Decodes serial packets received over a serial port.
     */
    class SerialPacketDecoder
    {
    public:
        /**
         * Decodes a packet into a deserialized packet object.
         * @param buffer The buffer containing the packet data.
         * @param length The length of the buffer.
         * @return The deserialized packet object or nullptr if the packet is invalid.
         */
        static SerialPacket *decode(uint8_t *buffer, uint16_t length)
        {
            // Unstuff the buffer
            static uint8_t *decodedBuffer = new uint8_t[MAX_BUFFER_SIZE];
            size_t newLength = ByteStuffer::decode(buffer, length, decodedBuffer);

            // Buffer Reader
            BufferReader reader(decodedBuffer, newLength);

            // Packet Data
            uint8_t type = reader.readUInt8();                // Packet Type
            uint8_t id = reader.readUInt8();                  // Packet ID
            uint16_t payloadSize = reader.readUInt16BE();     // Payload Size
            uint8_t *payload = reader.readBytes(payloadSize); // Payload
            uint8_t checksum = reader.readUInt8();            // Checksum

            // Check if the checksum is valid
            uint8_t calculatedChecksum = Checksum::calc(decodedBuffer, payloadSize + 4);
            if (checksum != calculatedChecksum)
            {
                NTLogger::logWarning("Packet " + std::to_string(id) + " has invalid checksum " +
                                     "(" + std::to_string(checksum) + " != " + std::to_string(calculatedChecksum) + ")");
                return nullptr;
            }

            // Create Temporary Packet to store payload
            static EncodedSerialPacket *tempPacket = new EncodedSerialPacket();
            tempPacket->id = id;
            tempPacket->type = (SerialPacketTypeID)type;
            tempPacket->payload = payload;
            tempPacket->payloadSize = payloadSize;

            // Deserialize the packet
            return SerialPacketTypes::deserialize(tempPacket);
        }

    private:
        static constexpr size_t MAX_BUFFER_SIZE = 256;
    };
}