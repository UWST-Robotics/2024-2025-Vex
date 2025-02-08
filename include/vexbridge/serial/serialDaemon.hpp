#pragma once

#include <cstdint>
#include <queue>
#include "../utils/daemon.hpp"
#include "pros/serial.hpp"
#include "pros/error.h"
#include "../utils/ntLogger.hpp"
#include "serialPacketDecoder.hpp"
#include "serialPacketEncoder.hpp"
#include "packets/genericAckPacket.hpp"
#include "packets/batchValuePacket.hpp"
#include "types/serialPacket.hpp"

namespace vexbridge
{
    /**
     * Background daemon that handles serial read and write operations.
     */
    class SerialDaemon : private Daemon
    {
    public:
        /**
         * Creates a new serial daemon.
         * @param port The VEX V5 port to connect to.
         */
        SerialDaemon(uint8_t port)
            : serial(port, BAUDRATE)
        {
        }

        /**
         * Adds a serial packet to the write queue.
         * @param packet The packet to write.
         */
        void writePacket(SerialPacket *packet)
        {
            // Check if the queue is full
            if (writeQueue.size() >= MAX_QUEUE_SIZE)
            {
                NTLogger::logWarning("Serial write queue is full.");
                return;
            }

            // Push the packet to the queue
            writeQueue.push(packet);
        }

    protected:
        void update() override
        {
            // Loop while there are packets to write
            while (!writeQueue.empty())
            {
                // Try to collapse the queue into a batch packet
                BatchValuePacket::tryCollapseQueue(writeQueue);

                // Get the next packet in the queue
                SerialPacket *packet = writeQueue.front();
                writeQueue.pop();

                // Time Round Trip
                uint32_t startTime = pros::millis();

                // Write the packet to the serial port
                try
                {
                    writePacketToSerial(packet);
                }
                catch (std::exception &e)
                {
                    NTLogger::logError("Exception while writing packet to serial: " + std::string(e.what()));
                }

                // Log Round Trip Time
                NTLogger::log("Ping: " + std::to_string(pros::millis() - startTime) + "ms");

                // Delete the packet from memory
                delete packet;
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
                // Flush the serial port
                int32_t flushRes = serial.flush();
                if (flushRes == PROS_ERR)
                {
                    NTLogger::logWarning("Failed to flush serial port.");
                    return -1;
                }

                // Write the packet to the serial port
                int32_t writeRes = serial.write(writeBuffer, packetSize);
                if (writeRes == PROS_ERR)
                {
                    NTLogger::logWarning("Failed to write packet " + std::to_string(packet->id) + " to serial port.");
                    return -1;
                }

                // Wait for an ack
                int ackRes = waitForAck(packet->id);

                // Handle Success
                if (ackRes == 0)
                    return 0;
            }

            // Log Error
            NTLogger::logWarning("Failed to send packet " + std::to_string(packet->id) +
                                 " after " + std::to_string(MAX_RETRIES) + " retries.");
            return -1;
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

                // Check if the packet is an ack
                GenericAckPacket *ackPacket = dynamic_cast<GenericAckPacket *>(packet);
                if (ackPacket == nullptr)
                {
                    NTLogger::logWarning("Received invalid packet while waiting for ack.");
                    continue;
                }

                // Check if the ack is for the correct packet
                if (ackPacket->targetID != packetID)
                {
                    NTLogger::logWarning("Received ack for wrong packet. (" +
                                         std::to_string(packetID) +
                                         " != " +
                                         std::to_string(ackPacket->targetID) +
                                         ")");
                    continue;
                }

                // Success
                return 0;
            }

            // Timeout
            NTLogger::logWarning("Timeout waiting for packet");
            return -1;
        }

        /**
         * Reads a packet from the serial port
         * @return The packet read from the serial port or nullptr if no packet is available.
         */
        SerialPacket *readPacket()
        {
            // Check if there is data to read
            int32_t readBufferSize = serial.get_read_avail();
            if (readBufferSize == PROS_ERR)
                return nullptr;
            if (readBufferSize == 0)
                return nullptr;

            // Read data from the serial port
            static uint8_t *readBuffer = new uint8_t[MAX_BUFFER_SIZE];
            int32_t bytesRead = serial.read(readBuffer, readBufferSize);
            if (bytesRead == PROS_ERR)
                return nullptr;

            // Flush the serial port
            int32_t flushRes = serial.flush();
            if (flushRes == PROS_ERR)
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
                // Check for the end of a packet (null byte)
                if (readQueue[i] != 0)
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
        static constexpr uint32_t TIMEOUT = 100;       // ms
        static constexpr uint32_t UPDATE_INTERVAL = 2; // ms
        static constexpr uint32_t BAUDRATE = 115200;
        static constexpr uint8_t MAX_RETRIES = 3;
        static constexpr uint8_t MAX_QUEUE_SIZE = 64;
        static constexpr size_t MAX_BUFFER_SIZE = 1024;
        static constexpr bool WAIT_FOR_ACK = true;

        pros::Serial serial;
        std::queue<SerialPacket *> writeQueue;
        std::vector<uint8_t> readQueue;
    };
}