#pragma once
#include "pros/adi.hpp"
#include "../utils/logger.hpp"
#include "structs/hardwareBase.hpp"
#include <string>

namespace devils
{
    /**
     * Represents a non-V5 pneumatic valve controlled by ADI ports.
     * See https://github.com/msoe-vex/pcb-design/tree/main/VEX%20Solenoid%20Driver%20V2%20Complete
     */
    class ADIPneumatic : private HardwareBase
    {
    public:
        /**
         * Creates a new pneumatic controlled by an ADI port.
         * @param name The name of the pneumatic (for logging purposes)
         * @param port The ADI port of the motor controller (from 1 to 8)
         */
        ADIPneumatic(std::string name, int8_t port)
            : HardwareBase(name, "ADIPneumatic", port),
              controller(abs(port))
        {
            isInverted = port < 0;
            if (errno != 0)
                reportFault("ADI port is invalid");
        }

        /**
         * Sets the state of the pneumatic.
         * @param isExtended True to extend the pneumatic, false to retract it.
         */
        void setExtended(bool isExtended)
        {
            // Invert the value if the port is inverted
            if (isInverted)
                isExtended = !isExtended;
            this->isExtended = isExtended;

            // Set the value and check for errors
            int32_t status = controller.set_value(isExtended);

            // Check for errors
            if (status != 1)
                reportFault("Set ADI value failed");
        }

        /**
         * Extends the pneumatic.
         */
        void extend()
        {
            setExtended(true);
        }

        /**
         * Retracts the pneumatic.
         */
        void retract()
        {
            setExtended(false);
        }

        /**
         * Checks if the pneumatic is extended.
         * @return True if the pneumatic is extended, false otherwise.
         */
        bool getExtended()
        {
            return isExtended;
        }

    private:
        const pros::adi::DigitalOut controller;

        bool isExtended = false;
        bool isInverted = false;
    };
}