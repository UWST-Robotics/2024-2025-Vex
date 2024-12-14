#pragma once

#include "../subsystems/WackerSystem.hpp"
#include "devils/devils.h"

namespace devils
{
    class AutoWackStep : public IAutoStep
    {
    public:
        AutoWackStep(WackerSystem &wacker, bool shouldExtend = true)
            : wacker(wacker), shouldExtend(shouldExtend)
        {
        }

        void doStep() override
        {
            wacker.setExtended(shouldExtend);
            pros::delay(ACTUATION_DELAY);
        }

    private:
        static constexpr double ACTUATION_DELAY = 500; // ms

        bool shouldExtend;
        WackerSystem &wacker;
    };
}