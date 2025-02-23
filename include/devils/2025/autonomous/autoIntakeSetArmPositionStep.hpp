#pragma once

#include "../subsystems/IntakeSystem.hpp"
#include "devils/devils.h"

namespace devils
{
    class AutoIntakeSetArmPositionStep : public AutoStep
    {
    public:
        AutoIntakeSetArmPositionStep(IntakeSystem &intake, IntakeSystem::ArmPosition position)
            : intake(intake), position(position)
        {
        }

        void onStart() override
        {
            intake.setArmPosition(position);
        }

        bool checkFinished() override
        {
            return true;
        }

    private:
        IntakeSystem &intake;
        IntakeSystem::ArmPosition position;
    };
}