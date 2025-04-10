#pragma once

#include <cstdint>
#include "../utils/daemon.hpp"
#include "serialSocket.hpp"

namespace vexbridge::serial
{
    /**
     * Opens a socket to a serial port.
     */
    class SerialFetchInterval : private Daemon
    {
    protected:
        void update() override
        {
            // Make fetch packet
            auto packet = std::make_shared<FetchValuesPacket>();
            packet->type = SerialPacketTypeID::FETCH_VALUES;
            packet->id = 0;

            // Write the packet to the serial port
            SerialSocket::writePacketToAll(packet);

            // Pause interval
            pros::delay(FETCH_INTERVAL);
        }

    private:
        static constexpr uint32_t FETCH_INTERVAL = 100; // ms
    };
}