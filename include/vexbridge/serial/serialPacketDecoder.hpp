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
#include "../utils/cobsEncoder.hpp"

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
            // Decode COBS
            static uint8_t *decodedBuffer = new uint8_t[MAX_BUFFER_SIZE];
            size_t newLength = COBSEncoder::decode(buffer, length, decodedBuffer);

            // Buffer Reader
            BufferReader reader(decodedBuffer, newLength);

            // Packet Data
            uint8_t type = reader.readUInt8();                // Packet Type
            uint8_t id = reader.readUInt8();                  // Packet ID
            uint16_t payloadSize = reader.readUInt16BE();     // Payload Size
            uint8_t *payload = reader.readBytes(payloadSize); // Payload
            uint16_t checksum = reader.readUInt16BE();        // Checksum

            // Check if the checksum is valid
            uint16_t calculatedChecksum = Checksum::calc(decodedBuffer, payloadSize + 4);
            if (checksum != calculatedChecksum)
            {
                NTLogger::logWarning("Packet " + std::to_string(id) + " has invalid checksum " +
                                     "(" + std::to_string(checksum) + " != " + std::to_string(calculatedChecksum) + ")");
                return nullptr;
            }

            // Create Packet
            EncodedSerialPacket *packet = new EncodedSerialPacket();
            packet->id = id;
            packet->type = (SerialPacketTypeID)type;
            packet->payload = payload;
            packet->payloadSize = payloadSize;

            // Deserialize the packet
            return SerialPacketTypes::deserialize(packet);
        }

    private:
        static constexpr size_t MAX_BUFFER_SIZE = 256;
    };
}