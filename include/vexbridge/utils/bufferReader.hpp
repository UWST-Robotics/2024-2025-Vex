#pragma once

#include <cstdint>
#include <cstddef>

namespace vexbridge
{
    /**
     * Reads data sequentially from a buffer.
     * Checks for buffer overflow.
     */
    class BufferReader
    {
    public:
        /**
         * Creates a new buffer reader.
         * @param buffer The buffer to read from.
         * @param length The length of the buffer.
         */
        BufferReader(uint8_t *buffer, size_t length)
            : buffer(buffer),
              length(length)
        {
        }

        /**
         * Sets the reader position
         * @param offset The new offset in bytes
         */
        void setOffset(size_t offset)
        {
            this->offset = offset;
        }

        /**
         * Gets the current reader position
         * @return The current offset in bytes
         */
        size_t getOffset()
        {
            return offset;
        }

        /**
         * Reads an unsigned 8-bit integer from the buffer.
         * @return The integer read from the buffer.
         */
        uint8_t readUInt8()
        {
            if (offset >= length)
                return 0;
            return buffer[offset++];
        }

        /**
         * Copies the next n bytes from the buffer into a new buffer.
         * @param length The number of bytes to copy.
         * @return The new buffer containing the copied bytes.
         */
        uint8_t *readBytes(uint16_t length)
        {
            uint8_t *bytes = new uint8_t[length];
            for (uint16_t i = 0; i < length; i++)
                bytes[i] = readUInt8();
            return bytes;
        }

        /**
         * Reads an unsigned 16-bit integer from the buffer in little-endian format.
         * @return The integer read from the buffer.
         */
        uint16_t readUInt16LE()
        {
            uint16_t value = readUInt8();
            value |= (uint16_t)readUInt8() << 8;
            return value;
        }

        /**
         * Reads an unsigned 16-bit integer from the buffer in big-endian format.
         * @return The integer read from the buffer.
         */
        uint16_t readUInt16BE()
        {
            uint16_t value = (uint16_t)readUInt8() << 8;
            value |= readUInt8();
            return value;
        }

        /**
         * Reads an signed 64-bit double from the buffer.
         * @return The double read from the buffer.
         */
        double readDouble()
        {
            double value = 0;
            uint8_t *bytes = readBytes(8);
            for (int i = 0; i < 8; i++)
                value += bytes[i] << (i * 8);
            return value;
        }

        /**
         * Reads a string from the buffer.
         * Reads a uint16_t (LE) length followed by the string.
         * @return The string read from the buffer.
         */
        std::string readString()
        {
            uint16_t stringLength = readUInt16LE();
            if (offset + stringLength > length)
                return "";

            std::string str = "";
            for (uint16_t i = 0; i < stringLength; i++)
                str += (char)readUInt8();

            return str;
        }

    private:
        uint8_t *buffer;
        size_t length;
        size_t offset = 0;
    };
}