#pragma once

#include "../subsystems/ConveyorSystem.hpp"
#include "devils/devils.h"

namespace devils
{
    class AutoGrabMogoStep : public AutoStep
    {
    public:
        AutoGrabMogoStep(ConveyorSystem &conveyor, bool shouldGrab = true)
            : conveyor(conveyor), shouldGrab(shouldGrab)
        {
        }

        void onStart() override
        {
            conveyor.setGoalGrabbed(shouldGrab);
            actuationTimer.start();
        }

        bool checkFinished() override
        {
            return actuationTimer.finished();
        }

    private:
        static constexpr double ACTUATION_DELAY = 500; // ms

        bool shouldGrab;
        Timer actuationTimer = Timer(ACTUATION_DELAY);
        ConveyorSystem &conveyor;
    };
}