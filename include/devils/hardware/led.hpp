#pragma once
#include "pros/adi.hpp"
#include "motor.hpp"
#include "../utils/logger.hpp"
#include <string>

namespace devils
{
    /**
     * Represents an LED connected to the ADI ports.
     */
    class LED
    {
    public:
        /**
         * Creates a new LED.
         * @param name The name of the LED (for logging purposes)
         * @param port The ADI port of the LED (from 1 to 8)
         */
        LED(std::string name, uint8_t port)
            : name(name),
              led(port)
        {
            if (errno != 0 && LOGGING_ENABLED)
                Logger::error(name + ": adi port is invalid");
        }

        /**
         * Turns the LED on
         */
        void enable()
        {
            int32_t status = led.set_value(false);
            if (status != 1 && LOGGING_ENABLED)
                Logger::error(name + ": led enable failed");
        }

        /**
         * Turns the LED off
         */
        void disable()
        {
            int32_t status = led.set_value(true);
            if (status != 1 && LOGGING_ENABLED)
                Logger::error(name + ": led disable failed");
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

    private:
        static constexpr bool LOGGING_ENABLED = true;

        std::string name;
        pros::ADIDigitalOut led;
    };
}