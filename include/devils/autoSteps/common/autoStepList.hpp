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
         * @param loopCount The number of times to loop through the steps.
         */
        AutoStepList(std::initializer_list<IAutoStep *> steps, int loopCount = 1)
            : steps(steps), loopCount(loopCount)
        {
        }

        /**
         * Executes all steps in the list as a pros task.
         * This function will block until all steps are complete.
         */
        void doStep() override
        {
            for (int i = 0; i < loopCount; i++)
            {
                for (int o = 0; o < steps.size(); o++)
                {
                    // Debug
                    NetworkTables::updateValue("CurrentStep", o);
                    NetworkTables::updateValue("CurrentLoop", i);
                    steps[o]->doStep();
                }
            }
        }

    private:
        std::vector<IAutoStep *> steps;
        int loopCount = 1;
    };
}