#pragma once

#include "../subsystems/WackerSystem.hpp"
#include "devils/devils.h"

namespace devils
{
    class AutoWackStep : public AutoStep
    {
    public:
        AutoWackStep(WackerSystem &wacker, bool shouldExtend = true)
            : wacker(wacker), shouldExtend(shouldExtend)
        {
        }

        void onStart() override
        {
            wacker.setExtended(shouldExtend);
            actuationTimer.start();
        }

        bool checkFinished() override
        {
            return actuationTimer.finished();
        }

    private:
        static constexpr double ACTUATION_DELAY = 500; // ms

        bool shouldExtend;
        Timer actuationTimer = Timer(ACTUATION_DELAY);
        WackerSystem &wacker;
    };
}