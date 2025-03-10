#pragma once

#include "autoStep.hpp"

namespace devils
{
    class AbsoluteStepConverter;

    /**
     * Represents a list of AutoSteps.
     */
    class AutoStepList : public AutoStep
    {
        friend class AbsoluteStepConverter;

    public:
        /**
         * Creates a new list of AutoSteps.
         * @param steps The steps to execute.
         */
        AutoStepList(std::initializer_list<AutoStep *> steps)
            : steps(steps)
        {
        }

        /**
         * Creates a new list of AutoSteps.
         * @param steps The steps to execute.
         */
        AutoStepList(std::vector<AutoStep *> steps)
            : steps(steps)
        {
        }

        void onStart()
        {
            index = 0;
            steps[index]->onStart();
        }

        void onUpdate()
        {
            // If we reached the end, stop
            if (checkFinished())
                return;

            // Update the Current Step
            steps[index]->onUpdate();

            // Check if the current step is finished
            if (steps[index]->checkFinished())
            {
                // Stop the Current Step
                steps[index]->onStop();

                // Abort if we reached the end
                index++;
                if (checkFinished())
                    return;

                // Start the Next Step
                steps[index]->onStart();

                // Update the Next Step
                onUpdate();
            }
        }

        void onStop()
        {
            // Stop the Current Step
            if (index < steps.size())
                if (steps[index] != nullptr)
                    steps[index]->onStop();
        }

        bool checkFinished() override
        {
            return index >= steps.size();
        }

        /**
         * Gets all the steps in the list.
         * @return The steps in the list.
         */
        std::vector<AutoStep *> &getAllSteps()
        {
            return steps;
        }

    private:
        // State
        int index = 0;

        // Params
        std::vector<AutoStep *> steps;
    };
}