#pragma once
#include "../subsystems/IntakeSystem.hpp"

namespace devils
{
    class AsyncIntakeStep : public AutoStep
    {
    public:
        AsyncIntakeStep(IntakeSystem &intake)
            : intake(intake)
        {
        }

        void onUpdate() override
        {
            intake.moveArmToPosition();
        }

    private:
        IntakeSystem &intake;
        bool pickupRing;
    };
}