#pragma once

#include "../ntObjectBase.hpp"
#include "../networkTables.hpp"
#include "../../odom/odomSource.hpp"
#include "../../geometry/units.hpp"

namespace devils
{
    class NTHardware : private NTObjectBase
    {
    public:
        NTHardware(
            const std::string name,
            const std::string type,
            const int8_t port)
            : name(name),
              type(type),
              port(port)
        {
            ntPrefix = "_hardware/" + type + "_" + std::to_string(abs(port));
        }

        void serialize() override
        {
            // Run health check
            checkHealth();

            // Update network table
            NetworkTables::updateValue(ntPrefix + "/name", name);
            NetworkTables::updateValue(ntPrefix + "/type", type);
            NetworkTables::updateIntValue(ntPrefix + "/port", port);

            // Serialize hardware
            serializeHardware(ntPrefix);
        }

    protected:
        /**
         * Reports a hardware fault.
         * Value is sent to the network table and logged.
         * @param fault The fault to set
         */
        void reportFault(std::string fault)
        {
            isFaulted = true;
            NetworkTables::updateValue(ntPrefix + "/faults", fault);
            if (LOGGING_ENABLED)
                Logger::error(name + ": " + fault);
        }

        /**
         * Clears any faults on the hardware.
         */
        void clearFaults()
        {
            isFaulted = false;
            NetworkTables::updateValue(ntPrefix + "/faults", "");
        }

        /**
         * Called every interval to serialize the hardware to the network table.
         * @param ntPrefix The network table prefix
         */
        virtual void serializeHardware(std::string &ntPrefix) = 0;

        /**
         * Called every interval to check the health of the hardware.
         * This should call `reportFault` if any fault is detected.
         */
        virtual void checkHealth() = 0;

    private:
        static constexpr bool LOGGING_ENABLED = true;

        const std::string name = "";
        const std::string type = "";
        const int8_t port = 0;

        std::string ntPrefix = "";
        bool isFaulted = false;
    };
}