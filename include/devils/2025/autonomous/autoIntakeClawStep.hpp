#pragma once

#include "../subsystems/IntakeSystem.hpp"
#include "devils/devils.h"

namespace devils
{
    class AutoIntakeClawStep : public AutoStep
    {
    public:
        AutoIntakeClawStep(IntakeSystem &intake, bool isGrabbed)
            : intake(intake), isGrabbed(isGrabbed)
        {
        }

        void onStart() override
        {
            intake.setClawGrabbed(isGrabbed);
        }

        bool checkFinished() override
        {
            return true;
        }

    private:
        IntakeSystem &intake;
        bool isGrabbed;
    };
}