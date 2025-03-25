#pragma once

#include <cstdint>
#include <cstddef>
#include "buffer.h"

namespace vexbridge::utils
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
         */
        BufferReader(const Buffer &buffer)
            : buffer(buffer)
        {
        }

        /**
         * Sets the reader position
         * @param offset The new offset in bytes
         */
        void setOffset(const size_t offset)
        {
            this->offset = offset;
        }

        /**
         * Gets the current reader position
         * @return The current offset in bytes
         */
        size_t getOffset() const
        {
            return offset;
        }

        /**
         * Gets the number of bytes available to read.
         * @return The number of bytes available to read.
         */
        size_t getBytesAvailable() const
        {
            return buffer.size() - offset;
        }

        /**
         * Reads an unsigned 8-bit integer from the buffer.
         * @return The integer read from the buffer.
         */
        uint8_t readUInt8()
        {
            if (offset >= buffer.size())
                return 0;
            return buffer[offset++];
        }

        /**
         * Copies the next n bytes from the buffer into a new buffer.
         * @param length The number of bytes to copy.
         * @return The new buffer containing the copied bytes.
         */
        Buffer readBytes(uint16_t length)
        {
            // Check for buffer overflow
            if (length > buffer.size() - offset)
                length = buffer.size() - offset;

            // Copy the bytes
            Buffer newBuffer;
            for (uint16_t i = 0; i < length; i++)
                newBuffer.push_back(readUInt8());
            return newBuffer;
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
         * Reads a double from the buffer in big-endian format.
         * @return The double read from the buffer.
         */
        double readDoubleBE()
        {
            uint64_t value = (uint64_t)readUInt8() << 56;
            value |= (uint64_t)readUInt8() << 48;
            value |= (uint64_t)readUInt8() << 40;
            value |= (uint64_t)readUInt8() << 32;
            value |= (uint64_t)readUInt8() << 24;
            value |= (uint64_t)readUInt8() << 16;
            value |= (uint64_t)readUInt8() << 8;
            value |= (uint64_t)readUInt8();
            return *(double *)&value;
        }

        /**
         * Reads a double from the buffer in little-endian format.
         * @return The double read from the buffer.
         */
        double readDoubleLE()
        {
            uint64_t value = (uint64_t)readUInt8();
            value |= (uint64_t)readUInt8() << 8;
            value |= (uint64_t)readUInt8() << 16;
            value |= (uint64_t)readUInt8() << 24;
            value |= (uint64_t)readUInt8() << 32;
            value |= (uint64_t)readUInt8() << 40;
            value |= (uint64_t)readUInt8() << 48;
            value |= (uint64_t)readUInt8() << 56;
            return *(double *)&value;
        }

        /**
         * Reads a float from the buffer in big-endian format.
         * @return The float read from the buffer.
         */
        float readFloatBE()
        {
            uint32_t value = (uint32_t)readUInt8() << 24;
            value |= (uint32_t)readUInt8() << 16;
            value |= (uint32_t)readUInt8() << 8;
            value |= (uint32_t)readUInt8();
            return *(float *)&value;
        }

        /**
         * Reads a float from the buffer in little-endian format.
         * @return The float read from the buffer.
         */
        float readFloatLE()
        {
            uint32_t value = (uint32_t)readUInt8();
            value |= (uint32_t)readUInt8() << 8;
            value |= (uint32_t)readUInt8() << 16;
            value |= (uint32_t)readUInt8() << 24;
            return *(float *)&value;
        }

        /**
         * Reads a string from the buffer.
         * Reads a uint16_t (BE) length followed by the string.
         * @return The string read from the buffer.
         */
        std::string readString16()
        {
            uint16_t stringLength = readUInt16BE();
            if (stringLength > buffer.size() - offset)
                stringLength = buffer.size() - offset;

            std::string str = "";
            for (uint16_t i = 0; i < stringLength; i++)
                str += (char)readUInt8();

            return str;
        }

        /**
         * Reads a string from the buffer.
         * Reads a uint8_t length followed by the string.
         * @return The string read from the buffer.
         */
        std::string readString8()
        {
            // Read the string length
            uint8_t stringLength = readUInt8();

            // Check for buffer overflow
            if (stringLength > buffer.size() - offset)
                stringLength = buffer.size() - offset;

            std::string str = "";
            for (uint8_t i = 0; i < stringLength; i++)
                str += (char)readUInt8();

            return str;
        }

        /**
         * Checks if there is more data to read.
         * @return True if there is more data to read.
         */
        bool hasData() const
        {
            return offset < buffer.size();
        }

    private:
        const Buffer &buffer;
        size_t offset = 0;
    };
}