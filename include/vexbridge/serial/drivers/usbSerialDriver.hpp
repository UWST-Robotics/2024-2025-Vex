#pragma once

#include "pros/serial.hpp"
#include "pros/error.h"
#include "SerialDriver.hpp"
#include "../../utils/sdkExtensions.h"

namespace vexbridge::serial
{
    /**
     * Interface for reading and writing data over the USB serial port.
     */
    class USBSerialDriver : public SerialDriver
    {
    public:
        USBSerialDriver(uint8_t port)
        {
        }

        bool write(uint8_t *buffer, int32_t length)
        {
            // Grab the mutex
            mutex.take(0);

            // Write packet to USB
            int32_t writeRes = vexSerialWriteBuffer(1, buffer, length);
            if (writeRes < 0)
                return false;

            // Release the mutex
            mutex.give();
        }

        int32_t read(uint8_t *buffer) override
        {
            // Grab the mutex
            mutex.take(0);

            // Iterate over all available characters
            uint32_t bytesRead = 0;
            while (true)
            {
                // Read a single character from the serial port
                int32_t charRead = vexSerialReadChar(1);
                if (charRead < 0)
                    break;

                // Add the character to the buffer
                buffer[bytesRead++] = (uint8_t)charRead;
            }

            // Release the mutex
            mutex.give();

            // Return the number of bytes read
            return bytesRead;
        }

    private:
        pros::Mutex mutex;
    };
}