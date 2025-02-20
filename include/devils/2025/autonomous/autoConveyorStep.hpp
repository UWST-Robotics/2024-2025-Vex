#pragma once

#include "../subsystems/ConveyorSystem.hpp"
#include "devils/devils.h"

namespace devils
{
    /**
     * Runs the conveyor system at a given speed.
     */
    class AutoConveyorStep : public AutoStep
    {
    public:
        AutoConveyorStep(ConveyorSystem &conveyor, double conveyorSpeed = 1.0)
            : conveyor(conveyor), conveyorSpeed(conveyorSpeed)
        {
        }

        void onUpdate() override
        {
            conveyor.moveAutomatic(conveyorSpeed);
        }

    private:
        double conveyorSpeed;
        ConveyorSystem &conveyor;
    };
}