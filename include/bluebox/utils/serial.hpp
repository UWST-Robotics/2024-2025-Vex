#pragma once

#include "pros/serial.hpp"
#include "cobs.hpp"

namespace devils
{
    class Serial
    {
    public:
        /**
         * Connects to a V5 serial port.
         * Communicates with RS-485 devices over the V5 ports on the brain.
         * @param port The port to connect to.
         * @param baudrate The baudrate to run the port at.
         */
        Serial(uint8_t port, uint32_t baudrate = DEFAULT_BAUDRATE)
            : serial(port, baudrate)
        {
        }

        /**
         * Adds a byte to the buffer
         * @param byte The byte to add to the buffer.
         */
        void pushByte(uint8_t byte)
        {
            buffer.push_back(byte);
        }

        /**
         * Pushes a signed 16-bit integer to the buffer.
         * @param value The value to push to the buffer.
         */
        void pushInt16(int16_t value)
        {
            pushByte(value >> 8);
            pushByte(value & 0xFF);
        }

        /**
         * Pushes a signed 32-bit integer to the buffer.
         * @param value The value to push to the buffer.
         */
        void pushInt32(int32_t value)
        {
            pushByte(value >> 24);
            pushByte((value >> 16) & 0xFF);
            pushByte((value >> 8) & 0xFF);
            pushByte(value & 0xFF);
        }

        /**
         * Pushes a signed 64-bit integer to the buffer.
         * @param value The value to push to the buffer.
         */
        void pushInt64(int64_t value)
        {
            pushByte(value >> 56);
            pushByte((value >> 48) & 0xFF);
            pushByte((value >> 40) & 0xFF);
            pushByte((value >> 32) & 0xFF);
            pushByte((value >> 24) & 0xFF);
            pushByte((value >> 16) & 0xFF);
            pushByte((value >> 8) & 0xFF);
            pushByte(value & 0xFF);
        }

        /**
         * Pushes a double to the buffer.
         * @param value The value to push to the buffer.
         */
        void pushDouble(double value)
        {
            pushInt64(*reinterpret_cast<int64_t *>(&value));
        }

        /**
         * Pushes an integer to the buffer.
         * @param value The value to push to the buffer.
         */
        void pushInt(int value)
        {
            pushInt32(value);
        }

        /**
         * Pushes a string to the buffer.
         * Appends the length of the string before the string.
         * @param value The string to push to the buffer.
         */
        void pushString(std::string value)
        {
            pushByte(value.size());
            for (char c : value)
                pushByte(c);
        }

        /**
         *  Encodes the buffer using COBS and writes it to the serial port.
         */
        void flush()
        {
            // Encode the buffer using COBS
            uint8_t output[buffer.size() + 1];
            COBS::encode(buffer.data(), buffer.size(), output);

            // Write the buffer to the serial port
            serial.write(output, sizeof(output));

            // Clear the buffer
            buffer.clear();
        }

    private:
        static constexpr uint32_t DEFAULT_BAUDRATE = 9600;

        std::vector<uint8_t> buffer;
        pros::Serial serial;
    };
}