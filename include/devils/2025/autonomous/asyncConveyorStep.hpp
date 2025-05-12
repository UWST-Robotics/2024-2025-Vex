#pragma once
#include "../subsystems/ConveyorSystem.hpp"
#include "../subsystems/MogoGrabSystem.hpp"

namespace devils
{
    class AsyncConveyorStep : public AutoStep
    {
    public:
        AsyncConveyorStep(ConveyorSystem &conveyor, MogoGrabSystem &mogoGrabber)
            : conveyor(conveyor),
              mogoGrabber(mogoGrabber)
        {
        }

        void onUpdate() override
        {
            conveyor.moveAutomatic(targetSpeed);
            conveyor.setMogoGrabbed(mogoGrabber.getMogoGrabbed());
        }

        /**
         * Sets the target speed of the conveyor.
         * @param speed The target speed of the conveyor.
         */
        void setTargetSpeed(double speed)
        {
            targetSpeed = speed;
        }

    private:
        ConveyorSystem &conveyor;
        MogoGrabSystem &mogoGrabber;
        double targetSpeed = 1.0;
    };
}