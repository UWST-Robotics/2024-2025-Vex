#pragma once

#include "../subsystems/ConveyorSystem.hpp"
#include "../subsystems/IntakeSystem.hpp"
#include "../subsystems/MogoGrabSystem.hpp"
#include "devils/devils.h"

namespace devils
{
    /**
     * Runs the conveyor system at a given speed.
     */
    class AutoConveyorStep : public AutoStep
    {
    public:
        AutoConveyorStep(ConveyorSystem &conveyor,
                         IntakeSystem &intake,
                         MogoGrabSystem &mogoGrabber,
                         double conveyorSpeed = 1.0)
            : conveyor(conveyor),
              intake(intake),
              mogoGrabber(mogoGrabber),
              conveyorSpeed(conveyorSpeed)
        {
        }

        void onUpdate() override
        {
            conveyor.moveAutomatic(conveyorSpeed);
            conveyor.setMogoGrabbed(mogoGrabber.isMogoGrabbed());
            conveyor.setArmLowered(intake.getArmPosition() == IntakeSystem::ArmPosition::BOTTOM_RING);
        }

        void onStop() override
        {
            conveyor.forceMove(0);
        }

    private:
        ConveyorSystem &conveyor;
        IntakeSystem &intake;
        MogoGrabSystem &mogoGrabber;
        double conveyorSpeed;
    };
}