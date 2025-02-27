#pragma once

#include "../subsystems/MogoGrabSystem.hpp"
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
         * @param mogoGrabber The mogo grabber system.
         * @param hasMogoStep The step to execute if the mogo has been grabbed.
         * @param noMogoStep The step to execute if the mogo has not been grabbed.
         */
        AutoMogoBranchStep(MogoGrabSystem &mogoGrabber, AutoStep *hasMogoStep, AutoStep *noMogoStep)
            : AutoBranchStep(hasMogoStep, noMogoStep),
              mogoGrabber(mogoGrabber)
        {
        }

        bool getCondition() override
        {
            return mogoGrabber.hasMogo();
        }

    private:
        MogoGrabSystem &mogoGrabber;
    };
}