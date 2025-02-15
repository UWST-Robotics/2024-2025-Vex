#pragma once

#include "../subsystems/ConveyorSystem.hpp"
#include "devils/devils.h"

namespace devils
{
    /**
     * Branches based on whether the mogo has been grabbed.
     */
    class AutoMogoBranchStep : public IAutoStep
    {
    public:
        /**
         * Creates a new mogo branch step.
         * @param conveyor The conveyor system to check.
         * @param hasMogoStep The step to execute if the mogo has been grabbed.
         * @param noMogoStep The step to execute if the mogo has not been grabbed.
         */
        AutoMogoBranchStep(ConveyorSystem &conveyor, IAutoStep *hasMogoStep, IAutoStep *noMogoStep)
            : conveyor(conveyor),
              hasMogoStep(hasMogoStep),
              noMogoStep(noMogoStep)
        {
        }

        void doStep() override
        {
            // TODO: Implement mogo sensor
            if (conveyor.goalGrabbed())
                hasMogoStep->doStep();
            else
                noMogoStep->doStep();
        }

    private:
        ConveyorSystem &conveyor;
        IAutoStep *hasMogoStep;
        IAutoStep *noMogoStep;
    };
}