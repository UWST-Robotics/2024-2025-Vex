#pragma once
#include "../subsystems/ConveyorSystem.hpp"
#include "../subsystems/MogoGrabSystem.hpp"

namespace devils
{
    class AsyncPauseConveyorStep : public AutoStep
    {
    public:
        AsyncPauseConveyorStep(ConveyorSystem &conveyor,
                               uint32_t delay,
                               uint32_t duration)
            : conveyor(conveyor),
              delay(delay),
              duration(duration)
        {
        }

        void onStart() override
        {
            delayTimer.setDuration(delay);
            delayTimer.start();

            durationTimer.setDuration(delay + duration);
            durationTimer.start();
        }

        void onUpdate() override
        {
            // Finished
            if (durationTimer.finished())
                conveyor.setPaused(false);

            // In Progress
            else if (delayTimer.finished())
                conveyor.setPaused(true);
        }

        bool checkFinished() override
        {
            return durationTimer.finished();
        }

    private:
        ConveyorSystem &conveyor;
        uint32_t delay;
        uint32_t duration;

        Timer delayTimer = Timer(0);
        Timer durationTimer = Timer(0);
    };
}