#pragma once

#include <cstdint>

namespace bluebox
{
    /**
     * Helper class for calculating checksums.
     */
    struct Checksum
    {
        /**
         * Calculates the checksum of a byte array using a simple sum algorithm.
         * @param buffer The buffer to calculate the checksum of.
         * @param length The length of the buffer.
         * @return The checksum of the buffer.
         */
        static uint16_t calc(uint8_t *buffer, uint16_t length)
        {
            uint16_t checksum = 0;
            for (uint16_t i = 0; i < length; i++)
                checksum += buffer[i];
            return checksum;
        }
    };
}