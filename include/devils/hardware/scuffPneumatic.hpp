#pragma once
#include "pros/adi.hpp"
#include "../utils/logger.hpp"
#include "../network/networkObject.hpp"
#include <string>

namespace devils
{
    /**
     * Represents a non-V5 pneumatic valve controlled by ADI ports.
     * See https://github.com/msoe-vex/pcb-design/tree/main/VEX%20Solenoid%20Driver%20V2%20Complete
     */
    class ScuffPneumatic : private INetworkObject
    {
    public:
        /**
         * Creates a new scuff pneumatic.
         * @param name The name of the pneumatic (for logging purposes)
         * @param port The ADI port of the motor controller (from 1 to 8)
         */
        ScuffPneumatic(std::string name, uint8_t port)
            : port(port),
              name(name),
              controller(port)
        {
            if (errno != 0 && LOGGING_ENABLED)
                Logger::error(name + ": adi port is invalid");
        }

        /**
         * Extends the pneumatic.
         */
        void extend()
        {
            int32_t status = controller.set_value(true);
            isExtended = true;
            if (status != 1 && LOGGING_ENABLED)
                Logger::error(name + ": pneumatic extend failed");
        }

        /**
         * Retracts the pneumatic.
         */
        void retract()
        {
            int32_t status = controller.set_value(false);
            isExtended = false;
            if (status != 1 && LOGGING_ENABLED)
                Logger::error(name + ": pneumatic retract failed");
        }

        /**
         * Checks if the pneumatic is extended.
         * @return True if the pneumatic is extended, false otherwise.
         */
        bool getExtended()
        {
            return isExtended;
        }

        void serialize() override
        {
            // Get Prefix
            std::string networkTableKey = NetworkTables::GetHardwareKey("adi", port);

            // Update Network Table
            NetworkTables::UpdateValue(networkTableKey + "/name", name);
            NetworkTables::UpdateValue(networkTableKey + "/type", "ScuffPneumatic");
            NetworkTables::UpdateValue(networkTableKey + "/isExtended", std::to_string(isExtended));
        }

    private:
        static constexpr bool LOGGING_ENABLED = false;

        uint8_t port;
        std::string name;
        pros::adi::DigitalOut controller;
        bool isExtended = false;
    };
}