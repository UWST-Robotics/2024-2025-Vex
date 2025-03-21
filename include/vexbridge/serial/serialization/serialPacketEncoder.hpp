#pragma once

#include <cstdint>
#include "allPacketTypes.hpp"
#include "packetTypes/common/serialPacketType.h"
#include "../utils/checksum.hpp"
#include "../utils/byteStuffer.hpp"

namespace vexbridge::serial
{
    /**
     * Encodes serial packets for transmission over a serial port.
     */
    struct SerialPacketEncoder
    {
        /**
         * Encodes a serializable packet into a complete packet.
         * @param packet The packet to encode.
         * @param buffer The buffer to write the encoded packet to.
         * @return The size of the encoded packet.
         */
        static Buffer encode(const SerialPacket &packet)
        {
            // Find the packet type
            SerialPacketType *packetType = AllPacketTypes::get(packet.type);
            if (packetType == nullptr)
                throw std::runtime_error("Unknown packet type " + std::to_string((uint8_t)packet.type));

            // Serialize the packet
            auto encodedPacket = packetType->serialize(packet);

            // Create the packet buffer
            Buffer packetBuffer;
            BufferWriter packerWriter(packetBuffer);

            // Packet
            packerWriter.writeUInt8((uint8_t)encodedPacket->type);                          // Type
            packerWriter.writeUInt8(encodedPacket->id);                                     // ID
            packerWriter.writeUInt16BE(encodedPacket->payload.size());                      // Payload Size
            packerWriter.writeBytes(encodedPacket->payload, encodedPacket->payload.size()); // Payload

            // Checksum
            uint8_t checksum = Checksum::calc(packetBuffer, packerWriter.getOffset());
            packerWriter.writeUInt8(checksum);

            // Byte Stuff
            return ByteStuffer::encode(packetBuffer);
        }
    };
}