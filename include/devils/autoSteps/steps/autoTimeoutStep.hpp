#pragma once
#include <string>
#include "pros/rtos.hpp"
#include "../autoStep.hpp"
#include "../../utils/timer.hpp"

namespace devils
{
    /**
     * Represents an autostep that aborts after a given duration.
     */
    class AutoTimeoutStep : public AutoStep
    {
    public:
        /**
         * Creates a new timeout step.
         * @param autoStep The step to execute.
         * @param duration The duration of the step in milliseconds.
         */
        AutoTimeoutStep(
            std::unique_ptr<AutoStep> autoStep,
            uint32_t duration)
            : autoStep(std::move(autoStep)),
              timer(duration)
        {
        }

        void onStart() override
        {
            // Start Timer
            timer.start();

            // Start Auto Step
            if (autoStep)
                autoStep->onStart();
        }

        void onUpdate() override
        {
            // Update Auto Step
            if (autoStep)
                autoStep->onUpdate();
        }

        void onStop() override
        {
            // Stop Timer
            timer.stop();

            // Stop Auto Step
            if (autoStep)
                autoStep->onStop();
        }

        bool checkFinished() override
        {
            // Check Timer
            if (timer.finished())
                return true;

            // Check Auto Step
            if (autoStep)
                return autoStep->checkFinished();

            // Otherwise, return true
            return true;
        }

    protected:
        // Params
        std::unique_ptr<AutoStep> autoStep;
        Timer timer;
    };
}