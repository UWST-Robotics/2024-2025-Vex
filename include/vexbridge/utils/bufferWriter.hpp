#pragma once

#include <cstdint>
#include <cstddef>
#include <string>

namespace vexbridge
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
         * @param length The length of the buffer.
         */
        BufferWriter(uint8_t *buffer, size_t length)
            : buffer(buffer),
              length(length)
        {
        }

        /**
         * Sets the write head position
         * @param offset The new offset in bytes
         */
        void setOffset(size_t offset)
        {
            this->offset = offset;
        }

        /**
         * Gets the current write head position
         * @return The current offset in bytes
         */
        size_t getOffset()
        {
            return offset;
        }

        /**
         * Writes an unsigned 8-bit integer to the buffer.
         * @param value The integer to write.
         */
        void writeUInt8(uint8_t value)
        {
            if (offset >= length)
                return;
            buffer[offset++] = value;
        }

        /**
         * Writes an unsigned 16-bit integer to the buffer in little-endian format.
         * @param value The integer to write.
         */
        void writeUInt16LE(uint16_t value)
        {
            if (offset + 1 >= length)
                return;
            buffer[offset++] = value & 0xFF;
            buffer[offset++] = value >> 8;
        }

        /**
         * Writes an unsigned 16-bit integer to the buffer in big-endian format.
         * @param value The integer to write.
         */
        void writeUInt16BE(uint16_t value)
        {
            if (offset + 1 >= length)
                return;
            buffer[offset++] = value >> 8;
            buffer[offset++] = value & 0xFF;
        }

        /**
         * Writes a byte array to the buffer.
         * @param bytes The bytes to write.
         */
        void writeBytes(uint8_t *bytes, uint16_t length)
        {
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
         * Writes the length of the string as a uint16_t (LE) followed by the string.
         * @param str The string to write.
         */
        void writeString(std::string str)
        {
            writeUInt16BE(str.length());
            for (uint16_t i = 0; i < str.length(); i++)
                writeUInt8(str[i]);
        }

    private:
        uint8_t *buffer;
        size_t length;
        size_t offset = 0;
    };
}