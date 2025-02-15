#pragma once
#include <string>
#include "pros/rtos.hpp"
#include "../common/autoStep.hpp"
#include "../../utils/timer.hpp"

namespace devils
{
    class AutoTimeoutStep : public AutoStep
    {
    public:
        /**
         * Performs the autostep, but aborts after a given duration.
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