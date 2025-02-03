#pragma once

#include <cstdint>
#include "packetType.hpp"

namespace bluebox
{
    /**
     * Represents a singular packet of data to be sent over a serial port.
     */
    class SerialPacket
    {
    public:
        /**
         * Constructs a new serial packet with the given type and ID.
         * @param type The type of the packet.
         */
        SerialPacket(PacketType type)
            : type(type)
        {
        }

        /**
         * Gets the type of the packet.
         * @return The type of the packet.
         */
        PacketType getPacketType()
        {
            return type;
        }

        /**
         * Assigns sequential ID for the packet.
         * Increments from 0 to 255 and then resets.
         * @return The ID of the packet.
         */
        static uint8_t getNextID()
        {
            static uint8_t idCounter = 0;
            return idCounter++;
        }

        /**
         * Sets the ID of the packet.
         * @param id The ID of the packet.
         */
        void setPacketID(uint8_t id)
        {
            this->id = id;
        }

        /**
         * Gets the ID of the packet.
         * @return The ID of the packet.
         */
        uint8_t getPacketID()
        {
            return id;
        }

        /**
         * Serializes the packet into a byte array.
         * @param buffer The buffer to serialize the packet into.
         * @return The size of the serialized packet.
         */
        virtual size_t serialize(uint8_t *buffer) = 0;

    private:
        PacketType type;
        uint8_t id;
    };
}