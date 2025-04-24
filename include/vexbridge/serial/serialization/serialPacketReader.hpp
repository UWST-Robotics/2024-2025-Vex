#pragma once
#include <vector>
#include <cstdint>
#include <memory>
#include "pros/rtos.hpp"
#include "../packetTypes/common/sentSerialPacket.h"
#include "../packetTypes/common/serialPacket.h"
#include "../packetTypes/common/encodedSerialPacket.h"
#include "../drivers/serialDriver.hpp"
#include "../serialization/serialPacketDecoder.hpp"
#include "../helpers/updateValuePacketHandler.hpp"
#include "../helpers/ackPacketHandler.hpp"

namespace vexbridge::serial
{
    // Forward declaration of SerialPacketWriter
    class SerialPacketWriter;

    /**
     * Handles reading and handling of serial packets from the serial port.
     */
    class SerialPacketReader
    {
    public:
        /**
         * Handles reading and handling of serial packets from the serial port.
         * @param serialDriver The serial driver to use for reading packets.
         * @throws std::runtime_error if the serial driver is nullptr.
         */
        SerialPacketReader(std::shared_ptr<SerialDriver> serialDriver)
            : serialDriver(serialDriver),
              serialWriter(nullptr)
        {
            if (!serialDriver)
                throw std::runtime_error("Serial driver cannot be nullptr.");
        }

        /**
         * Reads packets from the serial port and handles them.
         */
        void readPacketsFromSerial()
        {
            while (true)
            {
                // Read a packet from the serial port
                auto packet = readPacketFromSerial();
                if (!packet)
                    break; // No packet available

                // Handle the packet
                try
                {
                    // Handle Value Packets
                    UpdateValuePacketHandler::handlePacket(packet.get());

                    // Handle ACK Packets
                    AckPacketHandler::handlePacket(packet.get(), serialWriter.get());
                }
                catch (std::exception &e)
                {
                    // Log the error
                    // printf("Failed to handle packet: %s\n", e.what());
                }
            }
        }

        /**
         * Sets the serial writer to use for writing packets.
         * @param serialWriter The serial writer to use.
         */
        void setSerialWriter(std::shared_ptr<SerialPacketWriter> serialWriter)
        {
            this->serialWriter = serialWriter;
        }

    protected:
        /**
         * Reads the next packet from the serial port.
         * @return The packet read from the serial port or nullptr if no packet is available.
         */
        std::unique_ptr<SerialPacket> readPacketFromSerial()
        {
            // Read data from the serial port
            Buffer inputBuffer;
            int32_t bytesRead = serialDriver->read(inputBuffer);

            // Abort if no data was read
            if (bytesRead <= 0)
                return nullptr;

            // Append to the read buffer
            // This allows packets to be split across multiple read operations
            readBuffer.reserve(readBuffer.size() + bytesRead);
            readBuffer.insert(readBuffer.end(), inputBuffer.begin(), inputBuffer.end());

            // Trim the read buffer to prevent it from growing too large
            // Prevents memory leaks if the read buffer is never cleared (e.g. if no packets are found / garbage data is received)
            if (readBuffer.size() > MAX_BUFFER_SIZE)
                readBuffer.erase(readBuffer.begin(), readBuffer.begin() + readBuffer.size() - MAX_BUFFER_SIZE);

            // Iterate through the read buffer
            for (size_t i = 0; i < readBuffer.size(); i++)
            {
                // If the current flag is an escape flag, skip the next byte
                if (readBuffer[i] == ByteStuffer::ESCAPE_FLAG)
                {
                    i++;
                    continue;
                }

                // If the current flag is not the end flag, skip it
                if (readBuffer[i] != ByteStuffer::END_FLAG)
                    continue;

                // Decode the packet up to the end flag
                try
                {
                    // TODO: Only splice the read queue up to "i"
                    auto packet = SerialPacketDecoder::decode(readBuffer);

                    // Remove the current segment from the read queue
                    readBuffer.erase(readBuffer.begin(), readBuffer.begin() + i + 1);

                    return packet;
                }
                catch (std::exception &e)
                {
                    // Do nothing
                    // It's typical for the decoder to throw an exception if the packet got corrupt during transmission
                }

                // Remove the current segment from the read queue
                readBuffer.erase(readBuffer.begin(), readBuffer.begin() + i + 1);
            }

            // No packet found
            return nullptr;
        }

    private:
        /// @brief Maximum size of the read buffer. Prevents `readBuffer` from growing beyond hardware limits.
        static constexpr size_t MAX_BUFFER_SIZE = 2048; // Maximum size of the read buffer

        /// @brief Buffer of bytes read from the serial port
        Buffer readBuffer;

        /// @brief Serial hardware driver to use for reading packets
        std::shared_ptr<SerialDriver> serialDriver;

        /// @brief Writer to use for writing packets to the serial port
        std::shared_ptr<SerialPacketWriter> serialWriter;
    };
}