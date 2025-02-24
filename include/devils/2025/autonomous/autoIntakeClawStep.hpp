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
            actuationTimer.start();
        }

        bool checkFinished() override
        {
            return actuationTimer.finished();
        }

    private:
        static constexpr double ACTUATION_DELAY = 200; // ms

        IntakeSystem &intake;
        Timer actuationTimer = Timer(ACTUATION_DELAY);
        bool isGrabbed;
    };
}