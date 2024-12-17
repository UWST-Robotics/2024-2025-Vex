#pragma once

#include "utils/serial.hpp"
#include "utils/cobs.hpp"
#include <cstring>

namespace devils
{
    class NTSerial
    {
    public:
        /**
         * Serializes `NTValue` updates over a VEX V5 serial port.
         * @param port The V5 port to push data to.
         */
        NTSerial(uint8_t port)
            : serial(port)
        {
            instance = this;
        }

        /**
         * Resets the BlueBox.
         */
        void reset()
        {
            serial.pushByte(RESET_CMD);
            serial.flush();
        }

        /**
         * Labels a value ID
         * @param id The ID to label.
         * @param name The label for the ID.
         */
        void labelID(uint16_t id, const char *name)
        {
            serial.pushByte(LABEL_ID_CMD);
            serial.pushInt16(id);
            serial.pushString(name);
            serial.flush();
        }

        /**
         * Updates a boolean value.
         * @param id The ID of the value.
         * @param value The value to update.
         */
        void updateBoolean(uint16_t id, bool value)
        {
            serial.pushByte(UPDATE_BOOLEAN_CMD);
            serial.pushInt16(id);
            serial.pushByte(value ? 1 : 0);
            serial.flush();
        }

        /**
         * Updates an integer value.
         * @param id The ID of the value.
         * @param value The value to update.
         */
        void updateInt(uint16_t id, int value)
        {
            serial.pushByte(UPDATE_INT_CMD);
            serial.pushInt16(id);
            serial.pushInt32(value);
            serial.flush();
        }

        /**
         * Updates a double value.
         * @param id The ID of the value.
         * @param value The value to update.
         */
        void updateDouble(uint16_t id, double value)
        {
            serial.pushByte(UPDATE_DOUBLE_CMD);
            serial.pushInt16(id);
            serial.pushDouble(value);
            serial.flush();
        }

        /**
         * Updates a string value.
         * @param id The ID of the value.
         * @param value The value to update.
         */
        void updateString(uint16_t id, const char *value)
        {
            serial.pushByte(UPDATE_STRING_CMD);
            serial.pushInt16(id);
            serial.pushString(value);
            serial.flush();
        }

        static NTSerial *getInstance()
        {
            return instance;
        }

    private:
        static constexpr uint8_t RESET_CMD = 0x00;
        static constexpr uint8_t LABEL_ID_CMD = 0x01;
        static constexpr uint8_t UPDATE_BOOLEAN_CMD = 0x02;
        static constexpr uint8_t UPDATE_INT_CMD = 0x03;
        static constexpr uint8_t UPDATE_DOUBLE_CMD = 0x04;
        static constexpr uint8_t UPDATE_STRING_CMD = 0x05;

        // Singleton
        static NTSerial *instance;

        Serial serial;
    };
}

// Singleton instance
devils::NTSerial *devils::NTSerial::instance = nullptr;