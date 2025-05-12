#pragma once

#include "../../utils/runnable.hpp"
#include "../../utils/logger.hpp"

namespace devils
{
    class HardwareBase
    {
    public:
        /**
         * Base class for all hardware devices.
         * Handles network table serialization and fault reporting.
         * @param name The name of the hardware
         * @param type The type of the hardware (e.g. SmartMotor)
         * @param port The port of the hardware
         */
        HardwareBase(
            const std::string name,
            const std::string type,
            const int8_t port)
            : name(name)
        {
        }

    protected:
        /**
         * Reports a hardware fault.
         * Value is sent to the network table and logged.
         * @param fault The fault to set
         * @param code Error code (optional)
         */
        void reportFault(const std::string fault, const int code = 0)
        {
            // Log to console
            if (LOGGING_ENABLED)
                Logger::error(name + ": " + fault + " (" + std::to_string(code) + ")");
        }

        std::string name;

    private:
        static constexpr bool SERIALIZATION_ENABLED = true;
        static constexpr bool LOGGING_ENABLED = false;
    };
}