#pragma once
#include "pros/rtos.hpp"
#include "../common/autoStep.hpp"
#include "../../utils/timer.hpp"
#include "../../chassis/chassisBase.hpp"

namespace devils
{
    /**
     * Pauses the autonomous routine for a given duration.
     */
    class AutoPauseStep : public AutoStep
    {
    public:
        /**
         * Pauses the autonomous routine for a given duration.
         * @param duration The duration to pause in milliseconds.
         */
        AutoPauseStep(uint32_t duration)
            : timer(duration)
        {
        }

        void onStart() override
        {
            timer.start();
        }

        bool checkFinished() override
        {
            return timer.finished();
        }

    protected:
        Timer timer;
    };
}