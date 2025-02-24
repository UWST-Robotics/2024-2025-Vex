#pragma once

#include "../subsystems/ConveyorSystem.hpp"
#include "../subsystems/IntakeSystem.hpp"
#include "../subsystems/MogoGrabSystem.hpp"
#include "devils/devils.h"

namespace devils
{
    /**
     * Allows the conveyor to pickup rings
     */
    class AutoConveyorPickupStep : public AutoStep
    {
    public:
        AutoConveyorPickupStep(ConveyorSystem &conveyor, bool canPickupRings, uint32_t duration = 100)
            : conveyor(conveyor),
              canPickupRings(canPickupRings),
              durationTimer(duration)
        {
        }

        void onStart() override
        {
            durationTimer.start();
        }

        void onUpdate() override
        {
            conveyor.setPickupRing(canPickupRings);
        }

        void onStop() override
        {
            conveyor.setPickupRing(false);
        }

        bool checkFinished() override
        {
            return durationTimer.finished();
        }

    private:
        ConveyorSystem &conveyor;
        bool canPickupRings;
        Timer durationTimer;
    };
}