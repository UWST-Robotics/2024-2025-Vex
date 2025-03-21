#pragma once

#include "pros/serial.hpp"
#include "pros/error.h"
#include "SerialDriver.hpp"
#include "../utils/buffer.h"

using namespace vexbridge::utils;

// External C functions from the closed-source VEX V5 SDK
extern "C"
{
    /**
     * Writes a buffer to the serial port.
     * @param channel The channel to write to. Use 1 for stdout.
     * @param buffer The buffer to write.
     * @param length The length of the buffer.
     * @return The number of bytes written or -1 if an error occurred.
     */
    int32_t vexSerialWriteBuffer(uint32_t channel, uint8_t *buffer, uint32_t length);

    /**
     * Reads a character from the serial port.
     * @param channel The channel to read from. Use 1 for stdin.
     * @return The character read or -1 if no character is available.
     */
    int32_t vexSerialReadChar(uint32_t channel);
}

namespace vexbridge::serial
{
    /**
     * Interface for reading and writing data over the USB serial port.
     */
    class USBSerialDriver : public SerialDriver
    {
    public:
        USBSerialDriver()
        {
        }

        bool write(Buffer &buffer)
        {
            // Grab the mutex
            mutex.take(0);

            // Write packet to USB
            int32_t writeRes = vexSerialWriteBuffer(1, buffer.data(), buffer.size());
            if (writeRes < 0)
                return false;

            // Release the mutex
            mutex.give();
        }

        int32_t read(Buffer &buffer) override
        {
            // Grab the mutex
            mutex.take(0);

            // Iterate over all available characters
            while (true)
            {
                // Read a single character from the serial port
                int32_t charRead = vexSerialReadChar(1);
                if (charRead < 0)
                    break;

                // Add the character to the buffer
                buffer.push_back((uint8_t)charRead);
            }

            // Release the mutex
            mutex.give();

            // Return the number of bytes read
            return buffer.size();
        }

    private:
        pros::Mutex mutex;
    };
}