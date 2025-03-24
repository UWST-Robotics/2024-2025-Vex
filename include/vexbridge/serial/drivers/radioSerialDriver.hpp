#pragma once

#include "pros/link.hpp"
#include "pros/error.h"
#include "serialDriver.hpp"
#include <cmath>
#include "../../utils/buffer.h"

using namespace vexbridge::utils;

namespace vexbridge::serial
{
    /**
     * Interface for reading and writing data to another robot via a VEX V5 radio.
     */
    class RadioSerialDriver : public SerialDriver
    {
    public:
        /**
         * Creates a new radio serial driver.
         * @param port The port of the radio to use.
         * @param isTransmitter True if the radio is a transmitter, false if it is a receiver.
         */
        RadioSerialDriver(uint8_t port, bool isTransmitter)
            : serial(port, LINK_NAME, isTransmitter ? pros::E_LINK_TX : pros::E_LINK_RX)
        {
        }

        bool write(Buffer &buffer)
        {
            // Write the packet to the serial port
            uint32_t writeRes = serial.transmit_raw(buffer.data(), buffer.size());
            if (writeRes == PROS_ERR)
                return false;

            // Check if the entire packet was written
            if (writeRes != buffer.size())
                return false;

            // Return success
            return true;
        }

        int32_t read(Buffer &buffer) override
        {
            // Check if there is data to read
            uint32_t readBufferSize = serial.raw_receivable_size();
            if (readBufferSize == PROS_ERR)
                return -1;

            // Allocate buffer space
            buffer.resize(readBufferSize);

            // Read data from the serial port
            uint32_t bytesRead = serial.receive_raw(buffer.data(), readBufferSize);
            if (bytesRead == PROS_ERR)
                return -1;

            // Flush the serial port
            uint32_t flushRes = serial.clear_receive_buf();
            if (flushRes == PROS_ERR)
                return -1;

            // Return the number of bytes read
            return bytesRead;
        }

    private:
        const std::string LINK_NAME = "vexbridge";

        pros::Link serial;
    };
}