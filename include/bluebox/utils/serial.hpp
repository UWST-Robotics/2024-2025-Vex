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
         * Automatically encodes and decodes data using COBS.
         *
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
            writeBuffer.push_back(byte);
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
         * Reads from the buffer and decodes the COBS encoded data.
         * @return The decoded data or nullptr if no bytes are available.
         */
        uint8_t *read()
        {
            // Continue reading until there is no more data
            while (serial.get_read_avail())
            {
                // Get the next byte
                int32_t byte = serial.read_byte();
                readBuffer.push_back(byte);

                // Check if the byte is the end of the packet
                if (byte == 0x00)
                {
                    // Decode the buffer using COBS
                    uint8_t output[readBuffer.size()];
                    COBS::decode(readBuffer.data(), readBuffer.size(), output);

                    // Clear the buffer
                    readBuffer.clear();

                    // Return the output
                    return output;
                }
            }
        }

        /**
         * Checks if there is data available to read.
         * @return True if there is data available to read, false otherwise.
         */
        bool canRead()
        {
            return serial.get_read_avail() > 0;
        }

        /**
         *  Encodes the buffer using COBS and writes it to the serial port.
         */
        void flush()
        {
            // Encode the buffer using COBS
            uint8_t output[writeBuffer.size() + 1];
            COBS::encode(writeBuffer.data(), writeBuffer.size(), output);

            // Write the buffer to the serial port
            serial.write(output, sizeof(output));

            // Clear the buffer
            writeBuffer.clear();
        }

    private:
        static constexpr uint32_t DEFAULT_BAUDRATE = 9600;

        std::vector<uint8_t> writeBuffer;
        std::vector<uint8_t> readBuffer;
        pros::Serial serial;
    };
}