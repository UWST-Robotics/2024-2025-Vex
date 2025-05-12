#pragma once

#include <cstdint>
#include <cstddef>
#include <string>
#include <algorithm>
#include "buffer.h"

namespace vexbridge::utils
{
    /**
     * Writes data sequentially into a buffer.
     */
    class BufferWriter
    {
    public:
        /**
         * Creates a new buffer writer.
         * @param buffer The buffer to write to.
         */
        BufferWriter(Buffer &buffer)
            : buffer(buffer)
        {
        }

        /**
         * Gets the current write head position
         * @return The current offset in bytes
         */
        size_t getOffset() const
        {
            return buffer.size();
        }

        /**
         * Writes an unsigned 8-bit integer to the buffer.
         * @param value The integer to write.
         */
        void writeUInt8(uint8_t value)
        {
            buffer.push_back(value);
        }

        /**
         * Writes an unsigned 16-bit integer to the buffer in little-endian format.
         * @param value The integer to write.
         */
        void writeUInt16LE(uint16_t value)
        {
            buffer.push_back(value & 0xFF);
            buffer.push_back(value >> 8);
        }

        /**
         * Writes an unsigned 16-bit integer to the buffer in big-endian format.
         * @param value The integer to write.
         */
        void writeUInt16BE(uint16_t value)
        {
            buffer.push_back(value >> 8);
            buffer.push_back(value & 0xFF);
        }

        /**
         * Writes an float to the buffer in big-endian format.
         * @param value The float to write.
         */
        void writeFloatBE(float value)
        {
            uint8_t *bytes = (uint8_t *)&value;
            for (int i = 3; i >= 0; i--)
                writeUInt8(bytes[i]);
        }

        /**
         * Writes an float to the buffer in little-endian format.
         * @param value The float to write.
         */
        void writeFloatLE(float value)
        {
            uint8_t *bytes = (uint8_t *)&value;
            for (int i = 0; i < 4; i++)
                writeUInt8(bytes[i]);
        }

        /**
         * Writes a byte array to the buffer.
         * @param bytes The bytes to write.
         * @param length The number of bytes to write. Must be less than or equal to the length of the array.
         */
        void writeBytes(Buffer bytes, uint16_t length)
        {
            // Check if the length is greater than the array length
            if (length > bytes.size())
                length = bytes.size();

            // Write the bytes
            for (uint16_t i = 0; i < length; i++)
                writeUInt8(bytes[i]);
        }

        /**
         * Writes a double to the buffer in big-endian format.
         * @param value The double to write.
         */
        void writeDoubleBE(double value)
        {
            uint8_t *bytes = (uint8_t *)&value;
            for (int i = 7; i >= 0; i--)
                writeUInt8(bytes[i]);
        }

        /**
         * Writes a double to the buffer in little-endian format.
         * @param value The double to write.
         */
        void writeDoubleLE(double value)
        {
            uint8_t *bytes = (uint8_t *)&value;
            for (int i = 0; i < 8; i++)
                writeUInt8(bytes[i]);
        }

        /**
         * Writes the length of the string as a uint16_t (BE) followed by the string.
         * @param str The string to write.
         */
        void writeString16(std::string str)
        {
            uint16_t length = str.length();
            if (length > 0xFFFF)
                length = 0xFFFF;

            writeUInt16BE(length);
            for (uint16_t i = 0; i < length; i++)
                writeUInt8(str[i]);
        }

        /**
         * Writes the length of the string as a uint8_t followed by the string.
         * @param str The string to write.
         */
        void writeString8(std::string str)
        {
            uint8_t length = str.length();
            if (length > 0xFF)
                length = 0xFF;

            writeUInt8(length);
            for (uint8_t i = 0; i < length; i++)
                writeUInt8(str[i]);
        }

    private:
        Buffer &buffer;
    };
}