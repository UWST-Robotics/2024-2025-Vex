#pragma once

#include "../common/autoStep.hpp"

namespace devils
{
    /**
     * Executes an AutoStep asynchronously.
     * Remember to call `stopAll()` when the autonomous period ends to stop all async tasks.
     */
    class AutoAsyncStep : public AutoStep
    {
    public:
        AutoAsyncStep(AutoStep *asyncStep)
            : asyncStep(asyncStep)
        {
            allAsyncSteps.push_back(this);
        }
        ~AutoAsyncStep()
        {
            allAsyncSteps.erase(std::remove(allAsyncSteps.begin(), allAsyncSteps.end(), this), allAsyncSteps.end());
        }

        /**
         * Stops all async tasks.
         * Should be called when the autonomous period ends.
         */
        static void stopAll()
        {
            for (auto asyncStep : allAsyncSteps)
                if (asyncStep != nullptr)
                    asyncStep->forceStop();
        }

        /**
         * Immediately stops the async step mid-execution.
         */
        void forceStop()
        {
            asyncStep->stop();
        }

    protected:
        void onStart() override
        {
            auto task = asyncStep->runAsync();
        }

        bool checkFinished() override
        {
            return true;
        }

    private:
        // Store a list of all async steps
        static std::vector<AutoAsyncStep *> allAsyncSteps;

        AutoStep *asyncStep;
    };
}

// Initialize the static tasks vector
std::vector<devils::AutoAsyncStep *> devils::AutoAsyncStep::allAsyncSteps;