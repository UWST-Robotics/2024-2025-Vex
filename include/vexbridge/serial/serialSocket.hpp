#pragma once

#include <cstdint>
#include <queue>
#include <unordered_set>
#include <memory>
#include "../utils/daemon.hpp"
#include "../utils/globalInstances.hpp"
#include "pros/serial.hpp"
#include "pros/error.h"
#include "serialization/serialPacketDecoder.hpp"
#include "serialization/serialPacketEncoder.hpp"
#include "packetTypes/genericAckPacket.hpp"
#include "packetTypes/genericNAckPacket.hpp"
#include "packetTypes/resetPacket.hpp"
#include "drivers/serialDriver.hpp"
#include "helpers/updateValuePacketMerger.hpp"

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
        SerialSocket(std::unique_ptr<SerialDriver> serialDriver)
            : serial(std::move(serialDriver))
        {
            // Write Reset Packet
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
            // Check if the queue is full
            if (writeQueue.size() >= MAX_QUEUE_SIZE)
                throw std::runtime_error("Serial write queue is full.");

            // Check if the packet is nullptr
            if (!packet)
                throw std::runtime_error("Cannot write a nullptr packet.");

            // Attempt to merge an `UpdateValuePacket` into the queue
            if (UpdateValuePacketMerger::mergeInQueue(writeQueue, packet))
                return;

            // Push the packet to the queue
            writeQueue.push_back(packet);
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
            // Loop while there are packets to write
            while (!writeQueue.empty())
            {
                // Get the next packet in the queue
                auto packet = writeQueue.front();
                writeQueue.pop_front();

                // TODO: Attempt to merge equivalent packets into a `BatchPacket`

                // Write the packet to the serial port
                try
                {
                    writePacketToSerial(*packet.get());
                }
                catch (std::exception &e)
                {
                    // Do nothing
                    // It's typical for the write operation to fail if the SBC is disconnected/not replying
                }

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
        int writePacketToSerial(SerialPacket &packet)
        {
            // Generate a new Packet ID
            // Continuously cycles through 0-255 (1 byte)
            static uint8_t packetIDCounter = 0;
            packet.id = packetIDCounter++;

            // Serialize the packet
            Buffer writeBuffer = SerialPacketEncoder::encode(packet);

            // Loop until success or max retries
            for (int i = 0; i < MAX_RETRIES; i++)
            {
                // Write to Serial
                bool didWrite = serial->write(writeBuffer);
                if (!didWrite)
                    continue;

                // Wait for an ack
                // TODO: Async ack handling
                int ackRes = waitForAck(packet.id);
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
                auto packet = readPacket();

                // Check if a packet was read
                if (!packet)
                    continue;

                // Check if the packet is a nack
                GenericNAckPacket *nackPacket = dynamic_cast<GenericNAckPacket *>(packet.get());
                if (nackPacket != nullptr)
                    return -1; // <-- Exit to re-send the packet

                // Check if the packet is an ack
                GenericAckPacket *ackPacket = dynamic_cast<GenericAckPacket *>(packet.get());
                if (ackPacket == nullptr)
                    continue; // <-- Skip if not an ack

                // Check if the ack is for the correct packet
                if (ackPacket->id != packetID)
                    return -1; // <-- Exit to re-send the packet

                // Ack received
                return 0;
            }

            // Timeout
            return -1;
        }

        /**
         * Reads a packet from the serial port
         * @return The packet read from the serial port or nullptr if no packet is available.
         */
        std::unique_ptr<SerialPacket> readPacket()
        {
            // Read data from the serial port
            Buffer readBuffer;
            int32_t bytesRead = serial->read(readBuffer);

            // Abort if no data was read
            if (bytesRead <= 0)
                return nullptr;

            // Append to the read queue
            // This allows packets to be split across multiple read operations
            readQueue.reserve(readQueue.size() + bytesRead);
            readQueue.insert(readQueue.end(), readBuffer.begin(), readBuffer.end());

            // Trim the read queue to prevent it from growing too large
            // Prevents memory leaks if the read queue is never cleared (e.g. if no packets are found / garbage data is received)
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

                // Decode the packet up to the end flag
                try
                {
                    // TODO: Only splice the read queue up to "i"
                    auto packet = SerialPacketDecoder::decode(readQueue);
                    return packet;
                }
                catch (std::exception &e)
                {
                    // Do nothing
                    // It's typical for the decoder to throw an exception if the packet got corrupt during transmission
                }

                // Remove the current segment from the read queue
                readQueue.erase(readQueue.begin(), readQueue.begin() + i + 1);
            }

            // No packet found
            return nullptr;
        }

    private:
        static constexpr uint32_t TIMEOUT = 50;        // ms
        static constexpr uint32_t UPDATE_INTERVAL = 2; // ms
        static constexpr uint8_t MAX_RETRIES = 3;
        static constexpr size_t MAX_QUEUE_SIZE = 512;
        static constexpr size_t MAX_BUFFER_SIZE = 1024;
        static constexpr uint32_t POST_RECEIVE_DELAY = 4; // ms
        static constexpr bool WAIT_FOR_ACK = false;

        std::unique_ptr<SerialDriver> serial;
        std::deque<std::shared_ptr<SerialPacket>> writeQueue;
        std::vector<uint8_t> readQueue;
    };
}