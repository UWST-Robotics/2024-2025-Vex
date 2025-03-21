#pragma once

#include <cstdint>
#include "buffer.h"

namespace vexbridge::utils
{
    /**
     * Helper class for calculating checksums.
     */
    struct Checksum
    {
        /**
         * Calculates the 8-bit checksum of a byte array using a simple sum algorithm.
         * @param buffer The buffer to calculate the checksum of.
         * @param length The length to calculate the checksum over. Must be less than or equal to the buffer size.
         * @return The checksum of the buffer.
         */
        static uint8_t calc(const Buffer &buffer, const size_t length)
        {
            // Check if the length exceeds the buffer size
            if (length > buffer.size())
                throw std::runtime_error("Checksum length exceeds buffer size.");

            // Calculate the checksum
            uint8_t checksum = 0;
            for (size_t i = 0; i < length; i++)
                checksum += buffer[i];
            return checksum;
        }
    };
}