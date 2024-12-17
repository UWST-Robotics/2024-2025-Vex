#pragma once

#include <cstdint>
#include <cstdlib>

namespace devils
{
    class COBS
    {
    public:
        /**
         * Encodes a buffer using COBS.
         * @param src The source buffer to encode.
         * @param srcLength The length of the source buffer.
         * @param destination The destination buffer to write to.
         * @return The length of the encoded buffer.
         */
        static int encode(uint8_t *src, size_t srcLength, uint8_t *destination)
        {
            // TODO: Implement COBS encoding
            return 0;
        }

        static int decode(uint8_t *src, size_t srcLength, uint8_t *destination)
        {
            // TODO: Implement COBS decoding
            return 0;
        }
    };
}