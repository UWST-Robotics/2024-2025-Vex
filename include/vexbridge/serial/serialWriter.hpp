#pragma once

#include <cstdint>
#include <string>

namespace vexbridge::serial
{
    /**
     * Writes data to all active serial sockets.
     */
    struct SerialWriter
    {
        static void updateBoolean(uint16_t id, bool value)
        {
        }

        static void updateInt(uint16_t id, int32_t value)
        {
        }

        static void updateFloat(uint16_t id, float value)
        {
        }

        static void updateDouble(uint16_t id, double value)
        {
        }

        static void updateString(uint16_t id, std::string value)
        {
        }

        static void assignLabel(uint16_t id, std::string label)
        {
        }
    };
}