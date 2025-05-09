#pragma once

#include "../utils/runnable.hpp"
#include "../utils/logger.hpp"
#include <type_traits>
#include <memory>

namespace devils
{
    struct AutoStep : public Runnable, std::enable_shared_from_this<AutoStep>
    {
        void runAsync() override
        {
            // Add this step to the active steps list
            activeStepsMutex.take();
            activeSteps.push_back(shared_from_this());
            activeStepsMutex.give();

            // Run base class run method
            Runnable::runAsync();
        }

        /**
         * Stops all active runnables.
         */
        static void stopAll()
        {
            // Stop all active steps
            for (auto &step : activeSteps)
                step->stop();

            // Clear the active steps list
            activeStepsMutex.take();
            activeSteps.clear();
            activeStepsMutex.give();
        }

    private:
        static pros::Mutex activeStepsMutex; // Mutex to protect access to the active runnables vector
        static std::vector<std::shared_ptr<AutoStep>> activeSteps;
    };

    // Define the static members
    pros::Mutex AutoStep::activeStepsMutex;
    std::vector<std::shared_ptr<AutoStep>> AutoStep::activeSteps;

    // Define a shared pointer type for AutoStep
    typedef std::shared_ptr<AutoStep> AutoStepPtr;
}
