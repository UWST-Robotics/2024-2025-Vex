#pragma once

#include "devils/autoSteps/common/autoStep.hpp"

namespace devils
{
    /**
     * Represents a list of AutoSteps.
     */
    class AutoStepList : public IAutoStep
    {
    public:
        /**
         * Creates a new instance of AutoStepList.
         * @param steps The steps to execute.
         */
        AutoStepList(IAutoStep **steps)
            : steps(steps)
        {
        }

        /**
         * Executes all steps in the list as a pros task.
         * This function will block until all steps are complete.
         */
        void doStep() override
        {
            for (int i = 0; steps[i] != nullptr; i++)
                steps[i]->doStep();
        }

    private:
        IAutoStep **steps;
    };
}