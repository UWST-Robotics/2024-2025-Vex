#pragma once

#include <cstdint>
#include "../../utils/buffer.h"

using namespace vexbridge::utils;

namespace vexbridge::serial
{
    /**
     * Represents an interface for reading and writing data to a serial port.
     */
    struct SerialDriver
    {
        /**
         * Writes data to the serial port.
         * @param buffer Data is read from this buffer into the serial port.
         * @return True if the write was successful, false otherwise.
         */
        virtual bool write(Buffer &buffer) = 0;

        /**
         * Reads data from the serial port.
         * @param buffer Data is written to this buffer from the serial port.
         * @return The number of bytes read or -1 if an error occurred.
         */
        virtual int32_t read(Buffer &buffer) = 0;
    };
}