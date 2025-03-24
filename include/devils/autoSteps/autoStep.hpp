#pragma once

#include "../utils/runnable.hpp"
#include "vexbridge/utils/globalInstances.hpp"

namespace devils
{
    struct AutoStep : public Runnable, public GlobalInstances<AutoStep>
    {
        /**
         * Stops all active `AutoStep`s.
         * Should be called before teleoperated control.
         */
        static void stopAll()
        {
            for (auto &step : allInstances)
                step->stop();
        }
    };

    typedef std::unique_ptr<AutoStep> AutoStepPtr;
}