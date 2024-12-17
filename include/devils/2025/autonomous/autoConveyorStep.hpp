#pragma once

#include "../subsystems/ConveyorSystem.hpp"
#include "devils/devils.h"

namespace devils
{
    class AutoConveyorStep : public AutoStep
    {
    public:
        AutoConveyorStep(ConveyorSystem &conveyor, double conveyorSpeed = 1.0)
            : conveyor(conveyor), conveyorSpeed(conveyorSpeed)
        {
        }

        void onStart() override
        {
            conveyor.setAsyncSpeed(conveyorSpeed);
            conveyor.runAsync();
        }

        bool checkFinished() override
        {
            return true;
        }

    private:
        double conveyorSpeed;
        ConveyorSystem &conveyor;
    };
}