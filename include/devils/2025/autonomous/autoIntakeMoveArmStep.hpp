#pragma once

#include "../subsystems/IntakeSystem.hpp"
#include "devils/devils.h"

namespace devils
{
    class AutoIntakeMoveArmStep : public AutoStep
    {
    public:
        AutoIntakeMoveArmStep(IntakeSystem &intake)
            : intake(intake)
        {
        }

        void onUpdate() override
        {
            intake.moveArmToPosition();
        }

    private:
        IntakeSystem &intake;
    };
}