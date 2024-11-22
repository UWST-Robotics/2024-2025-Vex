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
         * Creates a new instance of AutoStepList.
         * @param steps The steps to execute.
         * @param loopCount The number of times to loop through the steps.
         */
        AutoStepList(std::vector<IAutoStep *> steps, int loopCount = 1)
            : steps(steps), loopCount(loopCount)
        {
        }

        /**
         * Destroys the AutoStepList and all steps in it.
         */
        ~AutoStepList()
        {
            // Steps were created with new, so delete them
            for (auto step : steps)
                delete step;
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
                    steps[o]->doStep();
                }
            }
        }

        /**
         * Gets all the steps in the list.
         * @return The steps in the list.
         */
        std::vector<IAutoStep *> &getAllSteps()
        {
            return steps;
        }

    private:
        std::vector<IAutoStep *> steps;
        int loopCount = 1;
    };
}