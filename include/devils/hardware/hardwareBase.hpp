#pragma once

#include "vexbridge/vexbridge.h"
#include "../utils/runnable.hpp"
#include "../utils/logger.hpp"

namespace devils
{
    class HardwareBase : private Runnable
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
            : Runnable(100),
              name(name),
              ntGroup("_hardware/" + name)
        //   ntType(ntGroup.makeValue("type", type))
        //   ntFault(ntGroup.makeValue<std::string>("fault", ""))
        {
            // TODO: Fix Stringed NT Types
            // Auto run serialization task
            if (SERIALIZATION_ENABLED)
                runAsync();
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
            // Set fault
            // ntFault.set(fault);

            // Log to console
            if (LOGGING_ENABLED)
                Logger::error(name + ": " + fault + " (" + std::to_string(code) + ")");
        }

        /**
         * Serializes the hardware to the network table.
         * Called on regular intervals.
         */
        virtual void serialize() = 0;

        std::string name;
        NTGroup ntGroup;
        // NTValue<std::string> ntType;
        // NTValue<std::string> ntFault;

    private:
        static constexpr bool SERIALIZATION_ENABLED = true;
        static constexpr bool LOGGING_ENABLED = false;

        void onUpdate() override
        {
            serialize();
        }
    };
}