#pragma once

#include <cstdint>
#include <stddef.h>
#include "buffer.h"

namespace vexbridge::utils
{
    struct ByteStuffer
    {
        static constexpr uint8_t START_FLAG = 0xAA;
        static constexpr uint8_t ESCAPE_FLAG = 0x92;
        static constexpr uint8_t END_FLAG = 0x00;

        /**
         * Encodes a buffer with the start flag and end flag.
         * Escapes any occurrences of the start flag and end flag with the escape flag.
         * @param input The input buffer.
         * @return The encoded output buffer.
         */
        static Buffer encode(const Buffer &input)
        {
            // Create the output buffer
            Buffer output;

            // Reserve worst-case space for the output buffer
            output.reserve(input.size() * 2);

            // Add the start flag to the output buffer
            output.push_back(START_FLAG);

            // Iterate over the input buffer
            for (size_t inputIndex = 0; inputIndex < input.size(); inputIndex++)
            {
                // If the current byte is the end flag, add the escape flag and the end flag to the output buffer
                if (input[inputIndex] == END_FLAG)
                {
                    output.push_back(ESCAPE_FLAG);
                    output.push_back(END_FLAG);
                }
                // If the current byte is the start flag, add the escape flag and the start flag to the output buffer
                else if (input[inputIndex] == START_FLAG)
                {
                    output.push_back(ESCAPE_FLAG);
                    output.push_back(START_FLAG);
                }
                // If the current byte is the escape flag, add 2 escape flags to the output buffer
                else if (input[inputIndex] == ESCAPE_FLAG)
                {
                    output.push_back(ESCAPE_FLAG);
                    output.push_back(ESCAPE_FLAG);
                }
                // Otherwise, add the current byte to the output buffer
                else
                {
                    output.push_back(input[inputIndex]);
                }
            }

            // Add the end flag to the output buffer
            output.push_back(END_FLAG);

            return output;
        }

        /**
         * Decodes a buffer using consistent overhead byte stuffing (COBS).
         * @param input The input buffer.
         * @return The decoded output buffer.
         */
        static Buffer decode(const Buffer &input)
        {
            // Create the output buffer
            Buffer output;

            // Reserve worst-case space for the output buffer
            output.reserve(input.size());

            // Iterate over the input buffer
            for (size_t inputIndex = 0; inputIndex < input.size(); inputIndex++)
            {
                // If the current byte is the start flag, reset and continue
                if (input[inputIndex] == START_FLAG)
                {
                    output.clear();
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
                    output.push_back(input[++inputIndex]);
                }
                // Otherwise, add the current byte to the output buffer
                else
                {
                    output.push_back(input[inputIndex]);
                }
            }

            return output;
        }
    };
}