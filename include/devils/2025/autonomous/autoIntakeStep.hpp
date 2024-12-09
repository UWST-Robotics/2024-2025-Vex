#pragma once

#include "../subsystems/IntakeSystem.hpp"
#include "devils/devils.h"

namespace devils
{
    class AutoIntakeStep : public IAutoStep
    {
    public:
        AutoIntakeStep(IntakeSystem &intake, double intakeSpeed = 1.0)
            : intake(intake), intakeSpeed(intakeSpeed)
        {
        }

        void doStep() override
        {
            intake.move(intakeSpeed);
        }

    private:
        double intakeSpeed;
        IntakeSystem &intake;
    };
}