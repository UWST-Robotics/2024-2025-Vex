#pragma once

#include <cstdint>
#include <queue>
#include "../utils/daemon.hpp"
#include "../utils/sdkExtensions.h"
#include "pros/serial.hpp"
#include "pros/error.h"
#include "serialQueue.hpp"
#include "serialPacketDecoder.hpp"
#include "serialPacketEncoder.hpp"
#include "packets/genericAckPacket.hpp"
#include "packets/genericNAckPacket.hpp"
#include "types/serialPacket.hpp"
#include "drivers/serialDriver.hpp"

namespace vexbridge::serial
{
    /**
     * Background daemon that handles serial read and write operations.
     */
    class SerialDaemon : private Daemon
    {
    public:
        /**
         * Creates a new serial daemon.
         * @param serialDriver The serial driver to use for reading and writing data.
         */
        SerialDaemon(SerialDriver *serialDriver)
            : serial(serialDriver)
        {
        }
        ~SerialDaemon()
        {
            delete serial;
        }

        /**
         * Adds a serial packet to the write queue.
         * Moves ownership of the packet to the queue.
         * @param packet The packet to write.
         */
        void writePacket(SerialPacket *packet)
        {
            writeQueue.push(packet);
        }

    protected:
        void update() override
        {
            // Loop while there are packets to write
            while (!writeQueue.empty())
            {
                // Get the next packet in the queue
                SerialPacket *packet = writeQueue.pop();
                if (packet == nullptr)
                    continue;

                // Stats
                uint32_t startTime = pros::millis();

                // Write the packet to the serial port
                try
                {
                    writePacketToSerial(packet);
                }
                catch (std::exception &e)
                {
                }

                // Delete the packet from memory
                delete packet;

                // Wait for SBC to pull RTS back down
                pros::delay(POST_RECEIVE_DELAY);
            }

            // Pause to prevent cpu overload
            pros::delay(UPDATE_INTERVAL);
        }

        /**
         * Writes a packet to the serial port.
         * Retries the write operation if it fails for up to MAX_RETRIES times.
         * @param packet The packet to write.
         * @return 0 if the packet was written successfully, -1 if the packet failed to write.
         */
        int writePacketToSerial(SerialPacket *packet)
        {
            // Set Packet ID
            static uint8_t packetIDCounter = 0;
            packet->id = packetIDCounter++;

            // Serialize the packet
            static uint8_t *writeBuffer = new uint8_t[MAX_BUFFER_SIZE];
            size_t packetSize = SerialPacketEncoder::encode(packet, writeBuffer);

            // Loop until success or max retries
            for (int i = 0; i < MAX_RETRIES; i++)
            {
                // Log
                // NTLogger::log("Writing packet " + std::to_string(packet->id) + " of type " + std::to_string((uint8_t)packet->type));

                // Write to Serial
                bool didWrite = serial->write(writeBuffer, (int32_t)packetSize);
                if (!didWrite)
                    continue;

                // Wait for an ack
                int ackRes = waitForAck(packet->id);
                if (ackRes != 0)
                    continue;
            }

            // Handle Failure
            throw std::runtime_error("Failed to write packet to serial port after " + std::to_string(MAX_RETRIES) + " retries.");
        }

        /**
         * Waits for an ack for a packet.
         * @param packetID The ID of the packet to wait for.
         * @return 0 if the ack was received, -1 if the ack was not received.
         */
        int waitForAck(uint16_t packetID)
        {
            // Check if we should wait for an ack
            if (!WAIT_FOR_ACK)
                return 0;

            // Get the start time
            uint32_t startTime = pros::millis();

            // Wait for ack or timeout
            while (pros::millis() - startTime < TIMEOUT)
            {
                // Pause to prevent cpu overload
                pros::delay(UPDATE_INTERVAL);

                // Read data from the serial port
                SerialPacket *packet = readPacket();
                if (packet == nullptr)
                    continue;

                // Check if the packet is a nack
                GenericNAckPacket *nackPacket = dynamic_cast<GenericNAckPacket *>(packet);
                if (nackPacket != nullptr)
                    return -1; // <-- Exit to re-send the packet

                // Check if the packet is an ack
                GenericAckPacket *ackPacket = dynamic_cast<GenericAckPacket *>(packet);
                if (ackPacket == nullptr)
                    continue; // <-- Skip if not an ack

                // Check if the ack is for the correct packet
                // TODO: Fix me!
                if (ackPacket->id != packetID)
                    return -1; // <-- Exit to re-send the packet

                return 0;
            }

            // Timeout
            return -1;
        }

        /**
         * Reads a packet from the serial port
         * @return The packet read from the serial port or nullptr if no packet is available.
         */
        SerialPacket *readPacket()
        {
            static uint8_t *readBuffer = new uint8_t[MAX_BUFFER_SIZE];
            int32_t bytesRead = serial->read(readBuffer);

            // Abort if no data was read
            if (bytesRead <= 0)
                return nullptr;

            // Append to the read queue
            // This allows packets to be split across multiple reads
            readQueue.insert(readQueue.end(), readBuffer, readBuffer + bytesRead);

            // Trim the read queue to prevent it from growing too large
            if (readQueue.size() > MAX_BUFFER_SIZE)
                readQueue.erase(readQueue.begin(), readQueue.begin() + readQueue.size() - MAX_BUFFER_SIZE);

            // Iterate through the read queue
            for (size_t i = 0; i < readQueue.size(); i++)
            {
                // If the current flag is an escape flag, skip the next byte
                if (readQueue[i] == ByteStuffer::ESCAPE_FLAG)
                {
                    i++;
                    continue;
                }

                // If the current flag is not the end flag, skip it
                if (readQueue[i] != ByteStuffer::END_FLAG)
                    continue;

                // Decode the packet up to the null byte
                SerialPacket *packet = SerialPacketDecoder::decode(&readQueue[0], i);

                // Remove the packet from the read queue
                readQueue.erase(readQueue.begin(), readQueue.begin() + i + 1);

                // Check if the packet is valid
                if (packet == nullptr)
                    continue;
                return packet;
            }

            // No packet found
            return nullptr;
        }

    private:
        static constexpr int32_t VEX_WRITE_BUFFER_SIZE = 1024;
        static constexpr uint32_t TIMEOUT = 50;        // ms
        static constexpr uint32_t UPDATE_INTERVAL = 2; // ms
        static constexpr uint32_t BAUDRATE = 115200;
        static constexpr uint8_t MAX_RETRIES = 3;
        static constexpr size_t MAX_QUEUE_SIZE = 512;
        static constexpr size_t MAX_BUFFER_SIZE = 1024;
        static constexpr double WRITE_DELAY_PER_BYTE = 0.08; // ms (estimate based off of 115200 baud)
        static constexpr uint32_t POST_RECEIVE_DELAY = 4;    // ms
        static constexpr bool WAIT_FOR_ACK = false;

        SerialDriver *serial = nullptr;
        SerialQueue writeQueue;
        std::vector<uint8_t> readQueue;
    };
}