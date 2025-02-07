#pragma once

#include <cstdint>
#include <stddef.h>

namespace vexbridge
{
    struct COBSEncoder
    {
        /**
         * Encodes a buffer using consistent overhead byte stuffing (COBS).
         * This ensures the output buffer does not contain any zero bytes except for the final byte.
         * @param input The input buffer.
         * @param length The length of the input buffer.
         * @param output The output buffer.
         * @return The length of the output buffer.
         */
        static size_t encode(uint8_t *input, size_t length, uint8_t *output)
        {
            // Offset of the code byte
            size_t codeOffset = 0;

            // Iterate through the input buffer
            for (size_t i = 0; i < length; i++)
            {
                size_t distance = i - codeOffset;

                // Check for zero byte
                if (input[i] == 0x00)
                {
                    // Write the code byte
                    output[codeOffset] = distance + 1;
                    codeOffset = i + 1;
                }

                // Check for max offset
                else if (distance == 0xFE)
                {
                    // Write the code byte
                    output[codeOffset] = 0xFF;
                    codeOffset = i + 1;
                }

                // Default case
                else
                {
                    // Copy the byte
                    output[i + 1] = input[i];
                }
            }

            // Write the final code byte
            output[codeOffset] = length - codeOffset + 1;

            // Write the final zero byte
            output[length + 1] = 0x00;

            // Return the length of the output buffer
            return length + 2;
        }

        /**
         * Decodes a buffer using consistent overhead byte stuffing (COBS).
         * @param input The input buffer.
         * @param length The length of the input buffer.
         * @param output The output buffer.
         * @return The length of the output buffer.
         */
        static size_t decode(uint8_t *input, size_t length, uint8_t *output)
        {
            // Copy the input buffer to the output buffer
            for (size_t i = 1; i < length; i++)
                output[i - 1] = input[i];

            // Iterate through the input buffer
            size_t offset = 0;
            while (offset < length)
            {
                // Get the code byte
                uint8_t code = input[offset];

                // Check for zero byte
                if (code == 0x00)
                    return offset;

                // Jump to the next code byte
                offset += code;

                // Check for max offset
                if (code == 0xFF)
                    continue;

                // Add null byte to output
                output[offset - 1] = 0x00;
            }

            // Return the length of the output buffer
            return length - 1;
        }
    };
}