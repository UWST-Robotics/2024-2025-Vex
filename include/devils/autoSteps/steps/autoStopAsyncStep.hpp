#pragma once

#include "autoAsyncStep.hpp"

namespace devils
{
    /**
     * Immediately stops the async step mid-execution.
     */
    class AutoStopAsyncStep : public AutoStep
    {
    public:
        AutoStopAsyncStep(AutoAsyncStep *asyncStep)
            : asyncStep(asyncStep)
        {
        }

    protected:
        void onStart() override
        {
            asyncStep->forceStop();
        }

        bool checkFinished() override
        {
            return true;
        }

    private:
        AutoAsyncStep *asyncStep;
    };
}