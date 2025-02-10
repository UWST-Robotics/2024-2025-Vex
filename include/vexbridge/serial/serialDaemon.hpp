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
#include "packets/genericNAckPacket.hpp"
#include "types/serialPacket.hpp"
#include "../utils/sdkExtensions.h"

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
         * @param port The VEX V5 port to connect to or 0 for USB.
         */
        SerialDaemon(uint8_t port)
        {
            if (port != 0)
                serial = new pros::Serial(port, BAUDRATE);
        }
        ~SerialDaemon()
        {
            if (serial != nullptr)
                delete serial;
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
                delete packet;
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
                // Get the next packet in the queue
                SerialPacket *packet = writeQueue.front();
                writeQueue.pop();

                // Stats
                uint32_t startTime = pros::millis();

                // Write the packet to the serial port
                try
                {
                    writePacketToSerial(packet);
                }
                catch (std::exception &e)
                {
                    NTLogger::logError("Writing packet to serial - " + std::string(e.what()));
                }

                // Log Stats
                NTLogger::log("Ping: " + std::to_string(pros::millis() - startTime) + "ms");

                // Delete the packet from memory
                delete packet;

                // Wait for SBC to pull RTS back down
                pros::delay(POST_TRANSMIT_DELAY);
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
                NTLogger::log("Writing packet " + std::to_string(packet->id) + " of type " + std::to_string((uint8_t)packet->type));

                // Serial Mode
                if (serial != nullptr)
                {
                    // Flush the serial port
                    serial->flush();

                    // Write the packet to the serial port
                    int32_t writeRes = serial->write(writeBuffer, packetSize);
                    if (writeRes == PROS_ERR)
                        throw std::runtime_error("Failed to write packet to serial port.");
                }

                // USB Mode
                else
                {
                    // Write packet to USB
                    int32_t writeRes = vexSerialWriteBuffer(1, &writeBuffer[0], packetSize);
                    if (writeRes < 0)
                        throw std::runtime_error("Failed to write packet to USB.");
                }

                // Wait for an ack
                int ackRes = waitForAck(packet->id);
                if (ackRes == 0)
                {
                    // Log Success
                    NTLogger::log("Packet " + std::to_string(packet->id) + " written successfully.");
                    return 0;
                }
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
                {
                    NTLogger::logWarning("Received nack while waiting for packet (" + std::to_string(packetID) + ")");
                    return -1; // <-- Exit to re-send the packet
                }

                // Check if the packet is an ack
                GenericAckPacket *ackPacket = dynamic_cast<GenericAckPacket *>(packet);
                if (ackPacket == nullptr)
                {
                    NTLogger::logWarning("Received invalid packet while waiting for ack.");
                    continue;
                }

                // Check if the ack is for the correct packet
                // TODO: Fix me!
                // if (ackPacket->id != packetID)
                // {
                //     NTLogger::logWarning("Received ack for wrong packet. (" +
                //                          std::to_string(packetID) +
                //                          " != " +
                //                          std::to_string(ackPacket->id) +
                //                          ")");
                //     continue;
                // }

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
            static uint8_t *readBuffer = new uint8_t[MAX_BUFFER_SIZE];
            int32_t bytesRead = 0;

            // Serial Mode
            if (serial != nullptr)
            {
                // Check if there is data to read
                int32_t readBufferSize = serial->get_read_avail();
                if (readBufferSize == PROS_ERR)
                    return nullptr;
                if (readBufferSize > MAX_BUFFER_SIZE)
                    readBufferSize = MAX_BUFFER_SIZE;

                // Read data from the serial port
                bytesRead = serial->read(readBuffer, readBufferSize);
                if (bytesRead == PROS_ERR)
                    return nullptr;

                // Flush the serial port
                int32_t flushRes = serial->flush();
                if (flushRes == PROS_ERR)
                    return nullptr;
            }

            // USB Mode
            else
            {
                // Read data from the serial port
                while (true)
                {
                    // TODO: Fix me!
                    // Read a character from the serial port
                    int32_t charRead = vexSerialReadChar(1);
                    if (charRead < 0)
                        break;
                    if (bytesRead >= MAX_BUFFER_SIZE)
                        break;

                    // Add the character to the buffer
                    readBuffer[bytesRead++] = (uint8_t)charRead;
                }
            }

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
                if (readQueue[i] == 0x92)
                {
                    i++;
                    continue;
                }

                // If the current flag is not the end flag, skip it
                if (readQueue[i] != 0xFF)
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
        static constexpr uint32_t TIMEOUT = 30;        // ms
        static constexpr uint32_t UPDATE_INTERVAL = 2; // ms
        static constexpr uint32_t BAUDRATE = 115200;
        static constexpr uint8_t MAX_RETRIES = 3;
        static constexpr size_t MAX_QUEUE_SIZE = 512;
        static constexpr size_t MAX_BUFFER_SIZE = 1024;
        static constexpr uint32_t POST_TRANSMIT_DELAY = 2; // ms
        static constexpr bool WAIT_FOR_ACK = true;

        pros::Serial *serial = nullptr;
        std::queue<SerialPacket *> writeQueue;
        std::vector<uint8_t> readQueue;
    };
}