#pragma once

#include <cstdint>
#include "allPacketTypes.hpp"
#include "../utils/bufferReader.hpp"
#include "../utils/checksum.hpp"
#include "../utils/byteStuffer.hpp"
#include "../utils/buffer.h"
#include "packetTypes/common/serialPacketType.h"

using namespace vexbridge::utils;

namespace vexbridge::serial
{
    /**
     * Decodes serial packets received over a serial port.
     */
    struct SerialPacketDecoder
    {
        /**
         * Decodes a packet into a deserialized packet object.
         * @param buffer The buffer containing the packet data.
         * @return The deserialized packet object or nullptr if the packet is invalid.
         */
        static std::unique_ptr<SerialPacket> decode(const Buffer &buffer)
        {
            // Unstuff the buffer
            Buffer decodedBuffer = ByteStuffer::decode(buffer);
            BufferReader reader(decodedBuffer);

            // Packet Data
            uint8_t type = reader.readUInt8();              // Packet Type
            uint8_t id = reader.readUInt8();                // Packet ID
            uint16_t payloadSize = reader.readUInt16BE();   // Payload Size
            Buffer payload = reader.readBytes(payloadSize); // Payload
            uint8_t checksum = reader.readUInt8();          // Checksum

            // Check if the checksum is valid
            uint8_t calculatedChecksum = Checksum::calc(decodedBuffer, payloadSize + 4);
            if (checksum != calculatedChecksum)
                throw std::runtime_error("Invalid checksum for packet " + std::to_string(id));

            // Create Temporary Packet to store payload
            EncodedSerialPacket tempPacket;
            tempPacket.id = id;
            tempPacket.type = (SerialPacketTypeID)type;
            tempPacket.payload = payload;

            // Find the packet type
            SerialPacketType *packetType = AllPacketTypes::get(tempPacket.type);
            if (packetType == nullptr)
                throw std::runtime_error("Unknown packet type " + std::to_string(type));

            // Deserialize the packet
            return packetType->deserialize(tempPacket);
        }
    };
}