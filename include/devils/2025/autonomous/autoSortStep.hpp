#pragma once

#include "../subsystems/ConveyorSystem.hpp"
#include "devils/devils.h"

namespace devils
{
    class AutoSortStep : public AutoStep
    {
    public:
        AutoSortStep(ConveyorSystem &conveyor, RingType ringType)
            : conveyor(conveyor),
              ringType(ringType)
        {
        }

        void onStart() override
        {
            conveyor.setRingSorting(ringType);
        }

        bool checkFinished() override
        {
            return true;
        }

    private:
        ConveyorSystem &conveyor;
        RingType ringType;
    };
}