#pragma once

#include "devils/devils.h"

namespace devils
{
    /**
     * Represents a robot.
     */
    class IntakeSystem
    {
    public:
        IntakeSystem(SmartMotorGroup &intakeMotors)
            : intakeMotors(intakeMotors)
        {
        }

        /**
         * Runs the intake system at a given voltage.
         * @param voltage The voltage to run the intake system at, from -1 to 1.
         */
        void move(double voltage)
        {
            intakeMotors.moveVoltage(voltage);
        }

    private:
        // TODO: Move pneumatic launcher to here
        SmartMotorGroup &intakeMotors;
    };
}