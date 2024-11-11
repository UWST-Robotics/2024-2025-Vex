#pragma once

#include "../subsystems/ConveyorSystem.hpp"
#include "devils/devils.h"

namespace devils
{
    class ConveyorStep : public IAutoStep
    {
    public:
        ConveyorStep(ConveyorSystem *conveyor, double intakeSpeed = 0.5)
            : conveyor(conveyor), intakeSpeed(intakeSpeed)
        {
        }

        void doStep() override
        {
            conveyor->tryMove(intakeSpeed);
        }

    private:
        double intakeSpeed;
        ConveyorSystem *conveyor;
    };
}