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
        AutoStepList(std::initializer_list<IAutoStep *> steps)
            : steps(steps)
        {
        }

        /**
         * Executes all steps in the list as a pros task.
         * This function will block until all steps are complete.
         */
        void doStep() override
        {
            for (int i = 0; i < steps.size(); i++)
            {
                // Debug
                NetworkTables::UpdateValue("CurrentStep", i);
                steps[i]->doStep();
            }
        }

    private:
        std::vector<IAutoStep *> steps;
    };
}