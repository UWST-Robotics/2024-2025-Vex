#pragma once
#include <vector>
#include <cstdint>
#include <memory>
#include "pros/rtos.hpp"
#include "../packetTypes/common/sentSerialPacket.h"
#include "../packetTypes/common/serialPacket.h"
#include "../packetTypes/common/encodedSerialPacket.h"
#include "../drivers/serialDriver.hpp"
#include "../serialization/serialPacketEncoder.hpp"

namespace vexbridge::serial
{
    // Forward declaration of SerialPacketReader
    class SerialPacketReader;

    /**
     * Handles sending and resending of packets to the serial port.
     */
    class SerialPacketWriter
    {
    public:
        /**
         * Handles sending and resending of serial packets.
         * @param serialDriver The serial driver to use for sending packets.
         * @throws std::runtime_error if the serial driver is nullptr.
         */
        SerialPacketWriter(std::shared_ptr<SerialDriver> serialDriver)
            : serialDriver(serialDriver),
              serialReader(nullptr)
        {
            if (!serialDriver)
                throw std::runtime_error("Serial driver cannot be nullptr.");
        }

        /**
         * Writes a packet to the serial port and handles acknowledgement and/or resending of the packet if needed.
         * @param serialPacket The packet to send.
         */
        void sendPacket(std::shared_ptr<SerialPacket> serialPacket)
        {
            // Check if the packet is nullptr
            if (!serialPacket)
                throw std::runtime_error("Cannot send a nullptr packet.");

            // Assign the packet ID
            static uint8_t packetID = 0;
            serialPacket->id = packetID++;

            // Write the packet to the serial port
            writePacketToSerial(serialPacket);

            // Create a new sent packet
            SentSerialPacket sentPacket;
            sentPacket.packet = serialPacket;
            sentPacket.timestamp = pros::millis();

            // Lock the mutex to prevent concurrent access
            sentPacketsMutex.take();

            // Add the packet to the sent packets list
            sentPackets.push_back(sentPacket);

            // Unlock the mutex after adding the packet
            sentPacketsMutex.give();
        }

        /**
         * Resends any packets that have not been acknowledged.
         * This should be called periodically to ensure that all packets are sent successfully.
         */
        void resendMissingPackets()
        {
            // Lock the mutex to prevent concurrent access
            sentPacketsMutex.take();

            // Iterate through all sent packets
            for (int i = 0; i < sentPackets.size(); i++)
            {
                // Get the iterator to the current packet
                auto it = sentPackets.begin() + i;
                if (it == sentPackets.end())
                    break; // Break if the iterator is at the end of the list

                // Check if the packet has timed out
                if (pros::millis() - it->timestamp < TIMEOUT)
                    continue;

                // Get the sent packet
                SentSerialPacket &sentPacket = *it;

                // Check if the packet has reached the max number of retries
                if (sentPacket.retries >= MAX_RETRIES)
                {
                    // Remove the packet from the list
                    sentPackets.erase(it);

                    // Log the failure
                    printf("Packet %d failed to send after %d retries.\n", sentPacket.packet->id, MAX_RETRIES);

                    // Decrement the index to account for the removed packet and continue
                    i--;
                    continue;
                }

                // Log the retry attempt
                printf("Resending packet %d (attempt %d)\n", sentPacket.packet->id, sentPacket.retries + 1);

                // Resend the packet
                writePacketToSerial(sentPacket.packet);

                // Update retries/timestamp
                sentPacket.retries++;
                sentPacket.timestamp = pros::millis();
            }

            // Unlock the mutex after processing all packets
            sentPacketsMutex.give();
        }

        /**
         * Marks a packet as acknowledged.
         * This should be called when a packet is acknowledged by the VEXBridge.
         * @param id The ID of the packet to acknowledge.
         */
        void ackPacket(uint8_t id)
        {
            // Lock the mutex to prevent concurrent access
            sentPacketsMutex.take();

            // Remove the packet from the sent packets list
            for (auto it = sentPackets.begin(); it != sentPackets.end(); ++it)
            {
                if (it->packet->id == id)
                {
                    sentPackets.erase(it);
                    break;
                }
            }

            // Unlock the mutex after processing all packets
            sentPacketsMutex.give();
        }

        /**
         * Sets the serial reader to use for reading packets.
         * @param serialReader The serial reader to use.
         */
        void setSerialReader(std::shared_ptr<SerialPacketReader> serialReader)
        {
            this->serialReader = serialReader;
        }

    protected:
        /**
         * Writes a packet to the serial port.
         * @param packet The packet to write.
         * @throws std::runtime_error if the packet is nullptr.
         */
        void writePacketToSerial(std::shared_ptr<SerialPacket> packet)
        {
            // Check if the packet is nullptr
            if (!packet)
                throw std::runtime_error("Cannot write a nullptr packet.");

            // Serialize the packet
            Buffer writeBuffer = SerialPacketEncoder::encode(*packet);
            if (writeBuffer.empty())
                throw std::runtime_error("Failed to serialize packet.");

            // Write to Serial
            serialDriver->write(writeBuffer);
        }

    private:
        /// @brief Maximum number of retries for sending a packet
        static constexpr uint8_t MAX_RETRIES = 3;

        /// @brief Maximum time to wait for a packet to be acknowledged (in milliseconds)
        static constexpr uint32_t TIMEOUT = 10;

        /// @brief List of packets that have been sent but not acknowledged
        std::vector<SentSerialPacket> sentPackets;

        /// @brief Mutex for synchronizing access to the sent packets list
        pros::Mutex sentPacketsMutex;

        /// @brief Serial hardware driver to use for sending packets
        std::shared_ptr<SerialDriver> serialDriver;

        /// @brief Serial packet reader to use for reading packets
        std::shared_ptr<SerialPacketReader> serialReader;
    };
}