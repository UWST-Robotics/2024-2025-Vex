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
            : ntGroup("_hardware/" + type + "_" + std::to_string(abs(port))),
              ntName(ntGroup.makeValue("name", name)),
              ntType(ntGroup.makeValue("type", type)),
              ntPort(ntGroup.makeValue("port", (int)port)),
              ntFault(ntGroup.makeValue("faults", (std::string) "")),
              ntFaultTimestamp(ntGroup.makeValue("faults_timestamp", 0))
        {
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
            // Send to network table
            ntFault.set(fault);
            ntFaultTimestamp.set(pros::millis());

            // Log to console
            if (LOGGING_ENABLED)
                Logger::error(ntName.get() + ": " + fault + " (" + std::to_string(code) + ")");
        }

        /**
         * Serializes the hardware to the network table.
         * Called on regular intervals.
         */
        virtual void serialize() = 0;

        NTGroup ntGroup;
        NTValue<std::string> ntName;
        NTValue<std::string> ntType;
        NTValue<std::string> ntFault;
        NTValue<int> ntFaultTimestamp;
        NTValue<int> ntPort;

    private:
        static constexpr bool SERIALIZATION_ENABLED = true;
        static constexpr bool LOGGING_ENABLED = false;

        void onUpdate() override
        {
            serialize();
        }
    };
}