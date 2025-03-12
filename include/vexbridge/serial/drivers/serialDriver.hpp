#pragma once

#include <cstdint>

namespace vexbridge::serial
{
    /**
     * Represents an interface for reading and writing data to a serial port.
     */
    struct SerialDriver
    {
        /**
         * Writes data to the serial port.
         * @param buffer The buffer to write to the serial port.
         * @param length The length of the buffer.
         * @return True if the write was successful, false otherwise.
         */
        virtual bool write(uint8_t *buffer, int32_t length) = 0;

        /**
         * Reads data from the serial port.
         * @param buffer The buffer to read from the serial port.
         * @return The number of bytes read or -1 if an error occurred.
         */
        virtual int32_t read(uint8_t *buffer) = 0;
    };
}