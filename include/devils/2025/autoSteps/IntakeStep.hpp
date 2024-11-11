#pragma once

#include "../subsystems/IntakeSystem.hpp"
#include "devils/devils.h"

namespace devils
{
    class IntakeStep : public IAutoStep
    {
    public:
        IntakeStep(IntakeSystem *intake, double intakeSpeed = 0.5)
            : intake(intake), intakeSpeed(intakeSpeed)
        {
        }

        void doStep() override
        {
            intake->move(intakeSpeed);
        }

    private:
        double intakeSpeed;
        IntakeSystem *intake;
    };
}