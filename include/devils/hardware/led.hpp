#pragma once
#include "pros/adi.hpp"
#include "motor.hpp"
#include "../utils/logger.hpp"
#include "../nt/objects/ntHardware.hpp"
#include <string>

namespace devils
{
    /**
     * Represents an LED connected to the ADI ports.
     */
    class LED : private NTHardware
    {
    public:
        /**
         * Creates a new LED.
         * @param name The name of the LED (for logging purposes)
         * @param port The ADI port of the LED (from 1 to 8)
         */
        LED(std::string name, uint8_t port)
            : NTHardware(name, "LED", port),
              led(port)
        {
            if (errno != 0)
                reportFault("Invalid port");
        }

        /**
         * Turns the LED on
         */
        void enable()
        {
            isEnabled = true;
            int32_t status = led.set_value(false);
            if (status != 1)
                reportFault("LED enable failed");
        }

        /**
         * Turns the LED off
         */
        void disable()
        {
            isEnabled = false;
            int32_t status = led.set_value(true);
            if (status != 1)
                reportFault("LED disable failed");
        }

        /**
         * Sets the LED to a boolean value. True is on, false is off.
         * @param enabled Whether or not the LED should be on
         */
        void setEnabled(bool enabled)
        {
            if (enabled)
                enable();
            else
                disable();
        }

        /**
         * Gets the state of the LED.
         * @return True if the LED is on, false if it is off.
         */
        bool getEnabled()
        {
            return isEnabled;
        }

    protected:
        void serializeHardware(std::string &ntPrefix) override
        {
            NetworkTables::UpdateValue(ntPrefix + "/enabled", isEnabled);
        }

        void checkHealth() override
        {
            clearFaults();
        }

    private:
        bool isEnabled = false;
        pros::ADIDigitalOut led;
    };
}