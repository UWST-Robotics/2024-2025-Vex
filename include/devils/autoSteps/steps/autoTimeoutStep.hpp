#pragma once
#include <string>
#include "pros/rtos.hpp"
#include "../common/autoStep.hpp"
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
            AutoStep *autoStep,
            uint32_t duration)
            : autoStep(autoStep),
              timer(duration)
        {
        }

        void onStart() override
        {
            // Start Timer
            timer.start();

            // Start Auto Step
            autoStep->onStart();
        }

        void onUpdate() override
        {
            // Update Auto Step
            autoStep->onUpdate();
        }

        void onStop() override
        {
            // Stop Timer
            timer.stop();

            // Stop Auto Step
            autoStep->onStop();
        }

        bool checkFinished() override
        {
            return timer.finished() || autoStep->checkFinished();
        }

    protected:
        // Params
        AutoStep *autoStep;
        Timer timer;
    };
}