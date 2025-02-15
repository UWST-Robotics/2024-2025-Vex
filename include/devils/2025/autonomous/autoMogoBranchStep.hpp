#pragma once

#include "../subsystems/ConveyorSystem.hpp"
#include "devils/devils.h"

namespace devils
{
    /**
     * Branches based on whether the mogo has been grabbed.
     */
    class AutoMogoBranchStep : public AutoBranchStep
    {
    public:
        /**
         * Creates a new mogo branch step.
         * @param conveyor The conveyor system to check.
         * @param hasMogoStep The step to execute if the mogo has been grabbed.
         * @param noMogoStep The step to execute if the mogo has not been grabbed.
         */
        AutoMogoBranchStep(ConveyorSystem &conveyor, AutoStep *hasMogoStep, AutoStep *noMogoStep)
            : AutoBranchStep(hasMogoStep, noMogoStep),
              conveyor(conveyor)
        {
        }

        bool getCondition() override
        {
            return conveyor.goalGrabbed();
        }

    private:
        ConveyorSystem &conveyor;
    };
}