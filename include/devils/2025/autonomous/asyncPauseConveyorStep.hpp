#pragma once
#include "../subsystems/ConveyorSystem.hpp"
#include "../subsystems/MogoGrabSystem.hpp"

namespace devils
{
    class AsyncPauseConveyorStep : public AutoStep
    {
    public:
        AsyncPauseConveyorStep(ConveyorSystem &conveyor,
                               uint32_t delay)
            : conveyor(conveyor),
              delay(delay)
        {
        }

        void onStart() override
        {
            delayTimer.setDuration(delay);
            delayTimer.start();
        }

        void onUpdate() override
        {
            if (delayTimer.finished())
                conveyor.setPaused(false);
        }

        void onStop() override
        {
            conveyor.setPaused(true);
        }

        bool checkFinished() override
        {
            return delayTimer.finished();
        }

    private:
        ConveyorSystem &conveyor;
        uint32_t delay;

        Timer delayTimer = Timer(0);
    };
}