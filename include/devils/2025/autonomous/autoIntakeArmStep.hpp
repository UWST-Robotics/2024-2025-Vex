#pragma once

#include "../subsystems/IntakeSystem.hpp"
#include "devils/devils.h"

namespace devils
{
    class AutoIntakeArmStep : public AutoStep
    {
    public:
        AutoIntakeArmStep(IntakeSystem &intake, IntakeSystem::ArmPosition position)
            : intake(intake), position(position)
        {
        }

        void onStart() override
        {
            intake.setArmPosition(position);
        }

        void onUpdate() override
        {
            intake.moveArmToPosition();
        }

        bool checkFinished() override
        {
            return intake.checkArmAtPosition();
        }

    private:
        IntakeSystem &intake;
        IntakeSystem::ArmPosition position;
    };
}