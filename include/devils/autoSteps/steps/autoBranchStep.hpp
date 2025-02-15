#pragma once

#include "../common/autoStep.hpp"

namespace devils
{
    /**
     * Base class to branch based on a condition.
     * Extend this class to create a new branch step.
     */
    class AutoBranchStep : public AutoStep
    {
    public:
        /**
         * Branches based on a condition.
         * @param trueStep The step to execute if the condition is true.
         * @param falseStep The step to execute if the condition is false.
         */
        AutoBranchStep(AutoStep *trueStep, AutoStep *falseStep)
            : trueStep(trueStep),
              falseStep(falseStep)
        {
        }

        /**
         * Gets the condition of the branch step.
         * Called once at the start of the step.
         * @return The condition of the branch step.
         */
        virtual bool getCondition() = 0;

        /**
         * Gets the active step based on the condition.
         * @return The active step based on the condition.
         */
        AutoStep *getActiveStep()
        {
            return condition ? trueStep : falseStep;
        }

    protected:
        void onStart() override
        {
            // Update the condition
            condition = getCondition();

            // Start the correct step
            getActiveStep()->onStart();
        }

        void onUpdate() override
        {
            // Update the correct step
            getActiveStep()->onUpdate();
        }

        void onStop() override
        {
            // Stop the correct step
            getActiveStep()->onStop();
        }

        bool checkFinished() override
        {
            // Check if the correct step is finished
            getActiveStep()->checkFinished();
        }

    private:
        bool condition = false;
        AutoStep *trueStep;
        AutoStep *falseStep;
    };
}