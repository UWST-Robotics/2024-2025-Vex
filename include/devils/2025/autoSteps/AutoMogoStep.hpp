#pragma once

#include "../subsystems/ConveyorSystem.hpp"
#include "devils/devils.h"

namespace devils
{
    class AutoGrabMogoStep : public IAutoStep
    {
    public:
        AutoGrabMogoStep(ConveyorSystem &conveyor, bool shouldGrab = true)
            : conveyor(conveyor), shouldGrab(shouldGrab)
        {
        }

        void doStep() override
        {
            conveyor.setGoalGrabbed(shouldGrab);
        }

    private:
        bool shouldGrab;
        ConveyorSystem &conveyor;
    };
}