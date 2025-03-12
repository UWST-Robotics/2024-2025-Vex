#pragma once

#include <cstdint>

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
         * @param length The length of the buffer.
         * @return The checksum of the buffer.
         */
        static uint8_t calc(uint8_t *buffer, size_t length)
        {
            uint8_t checksum = 0;
            for (size_t i = 0; i < length; i++)
                checksum += buffer[i];
            return checksum;
        }
    };
}