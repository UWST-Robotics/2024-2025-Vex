#pragma once

#include "../subsystems/IntakeSystem.hpp"
#include "devils/devils.h"

namespace devils
{
    class AutoIntakeStep : public AutoStep
    {
    public:
        AutoIntakeStep(IntakeSystem &intake, double intakeSpeed = 1.0)
            : intake(intake), intakeSpeed(intakeSpeed)
        {
        }

        void onStart() override
        {
            intake.move(intakeSpeed);
        }

        bool checkFinished() override
        {
            return true;
        }

    private:
        double intakeSpeed;
        IntakeSystem &intake;
    };
}