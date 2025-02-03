#pragma once

#include <cstring>
#include "serial/serialDaemon.hpp"
#include "serial/packets/updateValuePacket.hpp"
#include "serial/packets/updateLabelPacket.hpp"

namespace bluebox
{
    /**
     * Represents a serial port connection to the BlueBox.
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
        }
        ~NTSerial()
        {
            instance = nullptr;
        }

        /**
         * Resets the BlueBox.
         */
        void reset()
        {
            daemon.writePacket(new ResetPacket());
        }

        /**
         * Labels a value ID
         * @param id The ID to label.
         * @param name The label for the ID.
         */
        void labelID(uint16_t id, const char *name)
        {
            UpdateLabelPacket packet;
            packet.ntID = id;
            packet.label = name;
            daemon.writePacket(&packet);
        }

        /**
         * Updates a boolean value.
         * @param id The ID of the value.
         * @param value The value to update.
         */
        void updateBoolean(uint16_t id, bool value)
        {
            UpdateValuePacket packet;
            packet.ntID = id;
            packet.timestamp = pros::millis();
            packet.valueType = UpdateValuePacket::ValueType::BOOLEAN;
            packet.newValue = &value;
            daemon.writePacket(&packet);
        }

        /**
         * Updates an integer value.
         * @param id The ID of the value.
         * @param value The value to update.
         */
        void updateInt(uint16_t id, int value)
        {
            UpdateValuePacket packet;
            packet.ntID = id;
            packet.timestamp = pros::millis();
            packet.valueType = UpdateValuePacket::ValueType::INT;
            packet.newValue = &value;
            daemon.writePacket(&packet);
        }

        /**
         * Updates a double value.
         * @param id The ID of the value.
         * @param value The value to update.
         */
        void updateDouble(uint16_t id, double value)
        {
            UpdateValuePacket packet;
            packet.ntID = id;
            packet.timestamp = pros::millis();
            packet.valueType = UpdateValuePacket::ValueType::DOUBLE;
            packet.newValue = &value;
            daemon.writePacket(&packet);
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
bluebox::NTSerial *bluebox::NTSerial::instance = nullptr;