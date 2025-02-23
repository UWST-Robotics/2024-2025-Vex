#pragma once

#include "../subsystems/ConveyorSystem.hpp"
#include "devils/devils.h"

namespace devils
{
    /**
     * Runs the conveyor system at a given speed.
     */
    class AutoConveyorStep : public AutoStep
    {
    public:
        AutoConveyorStep(ConveyorSystem &conveyor, MogoGrabSystem &mogoGrabber, double conveyorSpeed = 1.0)
            : conveyor(conveyor),
              mogoGrabber(mogoGrabber),
              conveyorSpeed(conveyorSpeed)
        {
        }

        void onUpdate() override
        {
            conveyor.moveAutomatic(conveyorSpeed);
            conveyor.setMogoGrabbed(mogoGrabber.isMogoGrabbed());
        }

    private:
        ConveyorSystem &conveyor;
        MogoGrabSystem &mogoGrabber;
        double conveyorSpeed;
    };
}