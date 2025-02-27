#pragma once

#include "../subsystems/MogoGrabSystem.hpp"
#include "devils/devils.h"

namespace devils
{
    class AutoGrabMogoStep : public AutoStep
    {
    public:
        AutoGrabMogoStep(MogoGrabSystem &mogoGrabber, bool shouldGrab = true)
            : mogoGrabber(mogoGrabber), shouldGrab(shouldGrab)
        {
        }

        void onStart() override
        {
            mogoGrabber.setMogoGrabbed(shouldGrab);
            actuationTimer.start();
        }

        bool checkFinished() override
        {
            return actuationTimer.finished();
        }

    private:
        static constexpr double ACTUATION_DELAY = 200; // ms

        bool shouldGrab;
        Timer actuationTimer = Timer(ACTUATION_DELAY);
        MogoGrabSystem &mogoGrabber;
    };
}