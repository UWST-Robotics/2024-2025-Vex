#pragma once

#include "pros/serial.hpp"
#include "pros/error.h"
#include "serialDriver.hpp"
#include <cmath>
#include "../../utils/buffer.h"

using namespace vexbridge::utils;

namespace vexbridge::serial
{
    /**
     * Interface for reading and writing data to a VEX V5 serial port.
     * \deprecated VEX V5 ports are only half-duplex, so it is incompatible with VEXBridge's asynchronous serial protocol.
     */
    class VEXSerialDriver : public SerialDriver
    {
    public:
        VEXSerialDriver(uint8_t port)
            : serial(port)
        {
        }

        bool write(Buffer &buffer)
        {
            // Flush the serial port
            serial.flush();

            // Write the packet to the serial port
            int32_t writeRes = serial.write(buffer.data(), buffer.size());
            if (writeRes == PROS_ERR)
                return false;

            // Check if the entire packet was written
            if (writeRes != buffer.size())
                return false;

            // Wait for transmission to complete
            uint32_t writeDelay = ceil(WRITE_DELAY_PER_BYTE * buffer.size());
            pros::delay(writeDelay);

            // Return success
            return true;
        }

        int32_t read(Buffer &buffer) override
        {
            // Check if there is data to read
            int32_t readBufferSize = serial.get_read_avail();
            if (readBufferSize == PROS_ERR)
                return -1;

            // Allocate buffer space
            buffer.resize(readBufferSize);

            // Read data from the serial port
            int32_t bytesRead = serial.read(buffer.data(), readBufferSize);
            if (bytesRead == PROS_ERR)
                return -1;

            // Flush the serial port
            int32_t flushRes = serial.flush();
            if (flushRes == PROS_ERR)
                return -1;

            // Return the number of bytes read
            return bytesRead;
        }

    private:
        static constexpr double WRITE_DELAY_PER_BYTE = 0.08; // ms (estimate based off of 115200 baud)

        pros::Serial serial;
    };
}