#pragma once

#include <cstdint>
#include "serialPacket.hpp"
#include "serialChecksum.hpp"
#include "packets/updateValuePacket.hpp"
#include "packets/updateLabelPacket.hpp"
#include "packets/genericAckPacket.hpp"
#include "packets/resetPacket.hpp"

namespace bluebox
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
            NTLogger::log("Decoding packet of length " + std::to_string(length));
            // Check if the buffer is too small
            if (length < 10)
                return nullptr;

            // Header
            // 0xC9 0x36 0xB8 0x47

            // Type
            PacketType type = (PacketType)buffer[4];

            // ID
            uint8_t id = buffer[5];

            // Payload Size
            uint16_t payloadSize = (buffer[6] << 8) | buffer[7];
            if (length < payloadSize + 10)
                return nullptr;

            // Payload
            uint8_t *payload = &buffer[8];

            // Checksum
            uint16_t checksum = (buffer[payloadSize + 8] << 8) | buffer[payloadSize + 9];
            uint16_t calculatedChecksum = SerialChecksum::calc(buffer, payloadSize + 8);

            // Check if the checksum is valid
            // if (checksum != calculatedChecksum)
            //     return nullptr;

            NTLogger::log("Decoded packet of type " + std::to_string((uint8_t)type) + " with ID " + std::to_string(id));

            // Deserialize the packet
            return deserialize(type, id, payload, payloadSize);
        }

    private:
        /**
         * Deserializes a packet based on the packet type.
         * @param type The type of the packet.
         * @param id The ID of the packet.
         * @param payload The payload of the packet.
         * @param length The length of the payload.
         * @return The deserialized packet object or nullptr if the packet is invalid.
         */
        static SerialPacket *deserialize(PacketType type, uint8_t id, uint8_t *payload, uint16_t length)
        {
            // Deserialize the packet
            SerialPacket *packet = nullptr;
            switch (type)
            {
            case PacketType::UPDATE_LABEL:
                packet = UpdateLabelPacket::deserialize(payload, length);
                break;
            case PacketType::UPDATE_VALUE:
                packet = UpdateValuePacket::deserialize(payload, length);
                break;
            case PacketType::GENERIC_ACK:
                packet = GenericAckPacket::deserialize(payload, length);
                break;
            case PacketType::RESET:
                packet = new ResetPacket();
                break;
            default:
                NTLogger::logWarning("Received invalid packet type " + std::to_string((uint8_t)type));
                break;
            }

            // Assign the packet ID
            if (packet != nullptr)
                packet->setPacketID(id);

            return packet;
        }
    };
}