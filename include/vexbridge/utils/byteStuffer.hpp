#pragma once

#include <cstdint>
#include <stddef.h>

namespace vexbridge
{
    struct ByteStuffer
    {
        static constexpr uint8_t START_FLAG = 0xAF;
        static constexpr uint8_t ESCAPE_FLAG = 0x92;
        static constexpr uint8_t END_FLAG = 0xFA;

        /**
         * Encodes a buffer with the start flag and end flag.
         * Escapes any occurrences of the start flag and end flag with the escape flag.
         * @param input The input buffer.
         * @param length The length of the input buffer.
         * @param output The output buffer.
         * @return The length of the output buffer.
         */
        static size_t encode(uint8_t *input, size_t length, uint8_t *output)
        {
            size_t outputIndex = 0;
            size_t inputIndex = 0;

            // Add the start flag to the output buffer
            output[outputIndex++] = START_FLAG;

            while (inputIndex < length)
            {
                // If the current byte is the end flag, add the escape flag and the end flag to the output buffer
                if (input[inputIndex] == END_FLAG)
                {
                    output[outputIndex++] = ESCAPE_FLAG;
                    output[outputIndex++] = END_FLAG;
                }
                // If the current byte is the start flag, add the escape flag and the start flag to the output buffer
                else if (input[inputIndex] == START_FLAG)
                {
                    output[outputIndex++] = ESCAPE_FLAG;
                    output[outputIndex++] = START_FLAG;
                }
                // If the current byte is the escape flag, add 2 escape flags to the output buffer
                else if (input[inputIndex] == ESCAPE_FLAG)
                {
                    output[outputIndex++] = ESCAPE_FLAG;
                    output[outputIndex++] = ESCAPE_FLAG;
                }
                // Otherwise, add the current byte to the output buffer
                else
                {
                    output[outputIndex++] = input[inputIndex];
                }

                inputIndex++;
            }

            // Add the end flag to the output buffer
            output[outputIndex++] = END_FLAG;

            return outputIndex;
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
            size_t outputIndex = 0;
            size_t inputIndex = 0;

            while (inputIndex < length)
            {
                // If the current byte is the start flag, reset and continue
                if (input[inputIndex] == START_FLAG)
                {
                    outputIndex = 0;
                    inputIndex++;
                    continue;
                }
                // If the current byte is the end flag, break out of the loop
                if (input[inputIndex] == END_FLAG)
                {
                    break;
                }
                // If the current byte is the escape flag, add the next byte to the output buffer
                if (input[inputIndex] == ESCAPE_FLAG)
                {
                    output[outputIndex++] = input[++inputIndex];
                }
                // Otherwise, add the current byte to the output buffer
                else
                {
                    output[outputIndex++] = input[inputIndex];
                }

                inputIndex++;
            }

            return outputIndex;
        }
    };
}