#pragma once

#include "../subsystems/GoalRushSystem.hpp"
#include "devils/devils.h"

namespace devils
{
    class AutoGoalRushStep : public AutoStep
    {
    public:
        AutoGoalRushStep(GoalRushSystem &goalRush, bool isGrabbed)
            : goalRush(goalRush), isGrabbed(isGrabbed)
        {
        }

        void onStart() override
        {
            goalRush.setGoalRushExtended(isGrabbed);
            actuationTimer.start();
        }

        bool checkFinished() override
        {
            return actuationTimer.finished();
        }

    private:
        static constexpr double ACTUATION_DELAY = 150; // ms

        GoalRushSystem &goalRush;
        Timer actuationTimer = Timer(ACTUATION_DELAY);
        bool isGrabbed;
    };
}