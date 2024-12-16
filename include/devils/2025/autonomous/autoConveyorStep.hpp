#pragma once

#include "../subsystems/ConveyorSystem.hpp"
#include "devils/devils.h"

namespace devils
{
    class AutoConveyorStep : public IAutoStep
    {
    public:
        AutoConveyorStep(ConveyorSystem &conveyor, double conveyorSpeed = 1.0)
            : conveyor(conveyor), conveyorSpeed(conveyorSpeed)
        {
        }

        void doStep() override
        {
            conveyor.setAsyncSpeed(conveyorSpeed);
            conveyor.runAsync();
        }

    private:
        double conveyorSpeed;
        ConveyorSystem &conveyor;
    };
}