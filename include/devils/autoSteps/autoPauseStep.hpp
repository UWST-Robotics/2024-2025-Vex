#pragma once
#include "pros/rtos.hpp"
#include "common/autoStep.hpp"
#include "devils/chassis/chassisBase.hpp"

namespace devils
{
    /**
     * Represents a pause step in an autonomous routine.
     */
    class AutoPauseStep : public IAutoStep
    {
    public:
        /**
         * Creates a new pause step.
         * @param chassis The chassis to control.
         * @param duration The duration to pause in milliseconds.
         */
        AutoPauseStep(ChassisBase &chassis, double duration)
            : chassis(chassis),
              duration(duration)
        {
        }

        void doStep() override
        {
            // Stop the chassis
            chassis.stop();

            // Pause
            pros::delay(duration);
        }

    private:
        ChassisBase &chassis;
        double duration = 0;
    };
}