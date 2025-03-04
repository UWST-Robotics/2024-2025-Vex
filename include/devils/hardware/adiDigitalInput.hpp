#pragma once
#include "pros/adi.hpp"
#include "../utils/logger.hpp"
#include "structs/hardwareBase.hpp"
#include <string>

namespace devils
{
    /**
     * Represents a digital (on/off) input from the ADI ports.
     */
    class ADIDigitalInput : private HardwareBase
    {
    public:
        /**
         * Creates a new digital input controlled by an ADI port.
         * @param name The name of the input (for logging purposes)
         * @param port The ADI port of the motor controller (from 1 to 8)
         */
        ADIDigitalInput(std::string name, int8_t port)
            : HardwareBase(name, "ADIDigitalInput", port),
              controller(abs(port))
        {
            isInverted = port < 0;
            if (errno != 0)
                reportFault("ADI port is invalid");
        }

        /**
         * Gets the state of the digital input.
         * @return True if the input is high, false if the input is low.
         */
        bool getValue()
        {
            bool value = controller.get_value();
            if (isInverted)
                value = !value;
            return value;
        }

    protected:
        /**
         * Serializes the pneumatic to the network table.
         */
        void serialize() override
        {
            ntValue.set(getValue());
        }

    private:
        const pros::adi::DigitalIn controller;

        NTValue<bool> ntValue = ntGroup.makeValue("value", false);

        bool isInverted = false;
    };
}