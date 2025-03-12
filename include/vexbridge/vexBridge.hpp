#pragma once

#include <cstring>
#include "serial/serialDaemon.hpp"
#include "serial/packets/resetPacket.hpp"
#include "serial/packets/updateLabelPacket.hpp"
#include "serial/packets/updateBoolPacket.hpp"
#include "serial/packets/updateIntPacket.hpp"
#include "serial/packets/updateFloatPacket.hpp"
#include "serial/packets/updateDoublePacket.hpp"
#include "serial/packets/updateStringPacket.hpp"
#include "serial/packets/genericAckPacket.hpp"
#include "serial/packets/genericNAckPacket.hpp"
#include "serial/packets/logPacket.hpp"
#include "serial/packets/batchPacket.hpp"
#include "serial/packets/pingPacket.hpp"
#include "serial/packets/fetchPacket.hpp"
#include "serial/drivers/vexSerialDriver.hpp"

using namespace vexbridge::serial;

namespace vexbridge
{
    /**
     * Represents a serial port connection to the VEXBridge.
     * This is a singleton class - only one instance should be created.
     */
    class VEXBridge
    {
    public:
        /**
         * Creates a new VEXBridge instance.
         * @param port The VEX V5 port to connect to.
         */
        VEXBridge(uint8_t port) : daemon(new VEXSerialDriver(port))
        {
            if (instance != nullptr)
                delete instance;
            instance = this;

            // Call reset to clear NT
            reset();
        }
        ~VEXBridge()
        {
            instance = nullptr;
        }

        /**
         * Resets all values.
         */
        void reset()
        {
            ResetPacket *packet = new ResetPacket();
            packet->type = SerialPacketTypeID::RESET;
            daemon.writePacket(packet);
        }

        /**
         * Labels a value ID
         * @param id The ID to label.
         * @param name The label for the ID.
         */
        void labelID(uint16_t id, const char *name)
        {
            UpdateLabelPacket *packet = new UpdateLabelPacket();
            packet->type = SerialPacketTypeID::UPDATE_LABEL;
            packet->ntID = id;
            packet->label = name;
            daemon.writePacket(packet);
        }

        /**
         * Updates a boolean value.
         * @param id The ID of the value.
         * @param value The value to update.
         */
        void updateBoolean(uint16_t id, bool value)
        {
            UpdateBoolPacket *packet = new UpdateBoolPacket();
            packet->type = SerialPacketTypeID::UPDATE_BOOL;
            packet->ntID = id;
            packet->newValue = value;
            daemon.writePacket(packet);
        }

        /**
         * Updates an integer value.
         * @param id The ID of the value.
         * @param value The value to update.
         */
        void updateInt(uint16_t id, int value)
        {
            UpdateIntPacket *packet = new UpdateIntPacket();
            packet->type = SerialPacketTypeID::UPDATE_INT;
            packet->ntID = id;
            packet->newValue = value;
            daemon.writePacket(packet);
        }

        /**
         * Updates a float value.
         * @param id The ID of the value.
         * @param value The value to update.
         */
        void updateFloat(uint16_t id, float value)
        {
            UpdateFloatPacket *packet = new UpdateFloatPacket();
            packet->type = SerialPacketTypeID::UPDATE_FLOAT;
            packet->ntID = id;
            packet->newValue = value;
            daemon.writePacket(packet);
        }

        /**
         * Updates a double value.
         * @param id The ID of the value.
         * @param value The value to update.
         */
        void updateDouble(uint16_t id, double value)
        {
            UpdateDoublePacket *packet = new UpdateDoublePacket();
            packet->type = SerialPacketTypeID::UPDATE_DOUBLE;
            packet->ntID = id;
            packet->newValue = value;
            daemon.writePacket(packet);
        }

        /**
         * Updates a string value.
         * @param id The ID of the value.
         * @param value The value to update.
         */
        void updateString(uint16_t id, const char *value)
        {
            UpdateStringPacket *packet = new UpdateStringPacket();
            packet->type = SerialPacketTypeID::UPDATE_STRING;
            packet->ntID = id;
            packet->newValue = value;
            daemon.writePacket(packet);
        }

        static VEXBridge *getInstance()
        {
            return instance;
        }

    private:
        // Singleton
        static VEXBridge *instance;

        SerialDaemon daemon;
    };
}

// Singleton instance
vexbridge::VEXBridge *vexbridge::VEXBridge::instance = nullptr;

// Assign Packet Types
vexbridge::serial::SerialPacketType *vexbridge::serial::SerialPacketTypes::packetTypes[13] = {
    new ResetPacketType(),
    new UpdateLabelPacketType(),
    new FetchPacketType(),
    new LogPacketType(),
    new PingPacketType(),
    new GenericAckPacketType(),
    new GenericNAckPacketType(),

    new UpdateBoolPacketType(),
    new UpdateIntPacketType(),
    new UpdateFloatPacketType(),
    new UpdateDoublePacketType(),
    new UpdateStringPacketType(),

    new BatchPacketType()};