#pragma once
#include "pros/serial.hpp"
#include "../utils/cobs.hpp"
#include <cstring>

namespace devils
{
    // Forward declaration
    class NetworkTables;

    class BlueBox
    {
        // Let NetworkTables access protected members
        friend NetworkTables;

    public:
        /**
         * Listens to NT packets and sends them to the BlueBox.
         * @param port The port to listen on.
         */
        BlueBox(uint8_t port)
            : serial(port)
        {
        }

    protected:
        /**
         * Writes a buffer of data to the serial port.
         * Encodes the data using COBS.
         * @param dataBuffer The buffer of data to write.
         * @param length The length of the buffer.
         */
        void writeToSerial(uint8_t *dataBuffer, size_t length)
        {
            // Encode the data using COBS
            uint8_t encodedBuffer[length + 1];
            int encodedLength = COBS::encode(dataBuffer, length, encodedBuffer);

            // Write the encoded data to the serial port
            serial.write(encodedBuffer, encodedLength);
        }

        /**
         * Resets the BlueBox.
         */
        void reset()
        {
            uint8_t data[1] = {RESET_CMD};
            writeToSerial(data, 1);
        }

        /**
         * Labels a value ID
         * @param id The ID to label.
         * @param name The label for the ID.
         */
        void labelID(uint16_t id, const char *name)
        {
            size_t length = strlen(name);
            uint8_t data[4 + length];
            data[0] = LABEL_ID_CMD;
            data[1] = id >> 8;
            data[2] = id & 0xFF;
            data[3] = length;
            memcpy(data + 4, name, length);
            writeToSerial(data, 4 + length);
        }

        /**
         * Updates a boolean value.
         * @param id The ID of the value.
         * @param value The value to update.
         */
        void updateBoolean(uint16_t id, bool value)
        {
            uint8_t data[4] = {
                UPDATE_BOOLEAN_CMD,
                id >> 8,
                id & 0xFF,
                value};
            writeToSerial(data, 4);
        }

        /**
         * Updates an integer value.
         * @param id The ID of the value.
         * @param value The value to update.
         */
        void updateInt(uint16_t id, int value)
        {
            uint8_t data[6] = {
                UPDATE_INT_CMD,
                id >> 8,
                id & 0xFF,
                value >> 8,
                value & 0xFF};
            writeToSerial(data, 6);
        }

        /**
         * Updates a double value.
         * @param id The ID of the value.
         * @param value The value to update.
         */
        void updateDouble(uint16_t id, double value)
        {
            size_t doubleSize = sizeof(double);
            uint8_t data[6 + doubleSize];
            data[0] = UPDATE_DOUBLE_CMD;
            data[1] = id >> 8;
            data[2] = id & 0xFF;
            memcpy(data + 3, &value, doubleSize);
            writeToSerial(data, 6 + doubleSize);
        }

        /**
         * Updates a string value.
         * @param id The ID of the value.
         * @param value The value to update.
         */
        void updateString(uint16_t id, const char *value)
        {
            size_t length = strlen(value);
            uint8_t data[4 + length];
            data[0] = UPDATE_STRING_CMD;
            data[1] = id >> 8;
            data[2] = id & 0xFF;
            data[3] = length;
            memcpy(data + 4, value, length);
            writeToSerial(data, 4 + length);
        }

    private:
        static constexpr uint8_t RESET_CMD = 0x00;
        static constexpr uint8_t LABEL_ID_CMD = 0x01;
        static constexpr uint8_t UPDATE_BOOLEAN_CMD = 0x02;
        static constexpr uint8_t UPDATE_INT_CMD = 0x03;
        static constexpr uint8_t UPDATE_DOUBLE_CMD = 0x04;
        static constexpr uint8_t UPDATE_STRING_CMD = 0x05;

        pros::Serial serial;
    };
}