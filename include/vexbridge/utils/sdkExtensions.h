#include "stdint.h"

#ifndef VEXBRIDGE_SDK_EXTENSIONS_H
#define VEXBRIDGE_SDK_EXTENSIONS_H

// Methods defined here are part of the closed-source VEX V5 SDK
extern "C"
{
    /**
     * Writes a buffer to the serial port.
     * @param channel The channel to write to. Use 1 for stdout.
     * @param buffer The buffer to write.
     * @param length The length of the buffer.
     * @return The number of bytes written or -1 if an error occurred.
     */
    int32_t vexSerialWriteBuffer(uint32_t channel, uint8_t *buffer, uint32_t length);

    /**
     * Reads a character from the serial port.
     * @param channel The channel to read from. Use 1 for stdin.
     * @return The character read or -1 if no character is available.
     */
    int32_t vexSerialReadChar(uint32_t channel);
}

#endif // VEXBRIDGE_SDK_EXTENSIONS_H