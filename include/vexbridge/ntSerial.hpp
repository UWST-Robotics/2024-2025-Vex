#pragma once

#include <cstring>
#include "serial/serialDaemon.hpp"
#include "serial/packets/updateValuePacket.hpp"
#include "serial/packets/updateLabelPacket.hpp"

namespace vexbridge
{
    /**
     * Represents a serial port connection to the VEXBridge.
     * This is a singleton class - only one instance should be created.
     */
    class NTSerial
    {
    public:
        /**
         * Creates a new NTSerial instance.
         * @param port The VEX V5 port to connect to.
         */
        NTSerial(uint8_t port) : daemon(port)
        {
            if (instance != nullptr)
                delete instance;
            instance = this;

            // Call reset to clear NT
            reset();
        }
        ~NTSerial()
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
            UpdateValuePacket *packet = new UpdateValuePacket();
            packet->type = SerialPacketTypeID::UPDATE_VALUE;
            packet->ntID = id;
            packet->valueType = UpdateValuePacket::ValueType::BOOLEAN;
            packet->newValue = new bool(value);
            daemon.writePacket(packet);
        }

        /**
         * Updates an integer value.
         * @param id The ID of the value.
         * @param value The value to update.
         */
        void updateInt(uint16_t id, int value)
        {
            UpdateValuePacket *packet = new UpdateValuePacket();
            packet->type = SerialPacketTypeID::UPDATE_VALUE;
            packet->ntID = id;
            packet->valueType = UpdateValuePacket::ValueType::INT;
            packet->newValue = new int(value);
            daemon.writePacket(packet);
        }

        /**
         * Updates a float value.
         * @param id The ID of the value.
         * @param value The value to update.
         */
        void updateFloat(uint16_t id, float value)
        {
            UpdateValuePacket *packet = new UpdateValuePacket();
            packet->type = SerialPacketTypeID::UPDATE_VALUE;
            packet->ntID = id;
            packet->valueType = UpdateValuePacket::ValueType::FLOAT;
            packet->newValue = new float(value);
            daemon.writePacket(packet);
        }

        /**
         * Updates a double value.
         * @param id The ID of the value.
         * @param value The value to update.
         */
        void updateDouble(uint16_t id, double value)
        {
            UpdateValuePacket *packet = new UpdateValuePacket();
            packet->type = SerialPacketTypeID::UPDATE_VALUE;
            packet->ntID = id;
            packet->valueType = UpdateValuePacket::ValueType::DOUBLE;
            packet->newValue = new double(value);
            daemon.writePacket(packet);
        }

        /**
         * Updates a string value.
         * @param id The ID of the value.
         * @param value The value to update.
         */
        void updateString(uint16_t id, const char *value)
        {
            // TODO: Implement
        }

        static NTSerial *getInstance()
        {
            return instance;
        }

    private:
        // Singleton
        static NTSerial *instance;

        SerialDaemon daemon;
    };
}

// Singleton instance
vexbridge::NTSerial *vexbridge::NTSerial::instance = nullptr;