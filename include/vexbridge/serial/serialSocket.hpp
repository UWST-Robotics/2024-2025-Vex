#pragma once

#include "serialization/serialPacketWriter.hpp"
#include "serialization/serialPacketReader.hpp"
#include "../utils/daemon.hpp"
#include "../utils/globalInstances.hpp"

namespace vexbridge::serial
{
    /**
     * Opens a socket to a serial port.
     */
    class SerialSocket :

        // Runs `update` in a separate PROS task
        private Daemon,

        // Tracks all active serial sockets
        public GlobalInstances<SerialSocket>
    {
    public:
        /**
         * Creates a new serial daemon.
         * @param serialDriver The serial driver to use for reading and writing data.
         */
        SerialSocket(std::shared_ptr<SerialDriver> serialDriver)
        {
            // Initialize the serial writer and reader
            serialWriter = std::make_shared<SerialPacketWriter>(serialDriver);
            serialReader = std::make_shared<SerialPacketReader>(serialDriver);

            // Pass SerialWriter and SerialReader to each other
            serialWriter->setSerialReader(serialReader);
            serialReader->setSerialWriter(serialWriter);

            // Write `ResetPacket` on startup
            auto resetPacket = std::make_shared<ResetPacket>();
            resetPacket->type = SerialPacketTypeID::RESET;
            writePacket(resetPacket);
        }

        /**
         * Adds a serial packet to the write queue.
         * @param packet The packet to write.
         */
        void writePacket(std::shared_ptr<SerialPacket> packet)
        {
            try
            {
                // Check if the packet is nullptr
                if (!packet)
                    throw std::runtime_error("Cannot write a nullptr packet.");

                // Write the packet to the serial port
                serialWriter->sendPacket(packet);
            }
            catch (std::exception &e)
            {
                // Do nothing
            }
        }

        /**
         * Writes a packet to all active serial sockets.
         * @param packet The packet to write.
         */
        static void writePacketToAll(std::shared_ptr<SerialPacket> packet)
        {
            for (auto socket : allInstances)
                socket->writePacket(packet);
        }

    protected:
        void update() override
        {
            // Read packets from the serial port
            serialReader->readPacketsFromSerial();

            // Resend any packets that have not been acknowledged
            serialWriter->resendMissingPackets();

            // Pause to prevent cpu overload
            pros::delay(UPDATE_INTERVAL);
        }

    private:
        static constexpr uint32_t MAX_QUEUE_SIZE = 128; // Maximum number of packets in the write queue
        static constexpr uint32_t UPDATE_INTERVAL = 2;  // ms

        std::shared_ptr<SerialPacketWriter> serialWriter;
        std::shared_ptr<SerialPacketReader> serialReader;
    };
}