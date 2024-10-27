#pragma once
#include "pros/motors.hpp"
#include "../utils/logger.hpp"
#include <string>

namespace devils
{
    /**
     * Represents some kind of motor or motor group.
     */
    struct IMotor
    {
        /**
         * Runs the motor in voltage mode.
         * @param voltage The voltage to run the motor at, from -1 to 1.
         */
        virtual void moveVoltage(double voltage) = 0;

        /**
         * Stops the motor.
         */
        virtual void stop() = 0;

        /**
         * Gets the current position of the motor in encoder ticks.
         * @return The current position of the motor in encoder ticks.
         */
        virtual double getPosition() = 0;
    };
}