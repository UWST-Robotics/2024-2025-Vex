#pragma once

#include "../subsystems/ConveyorSystem.hpp"
#include "devils/devils.h"

namespace devils
{
    class AutoConveyorStep : public IAutoStep
    {
    public:
        AutoConveyorStep(ConveyorSystem &conveyor, double intakeSpeed = 0.5)
            : conveyor(conveyor), intakeSpeed(intakeSpeed)
        {
        }

        void doStep() override
        {
            conveyor.runAutomatic();
        }

    private:
        double intakeSpeed;
        ConveyorSystem &conveyor;
    };
}