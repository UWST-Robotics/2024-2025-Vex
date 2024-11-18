#pragma once

#include <vector>
#include "../geometry/pose.hpp"
#include "structs/trajectoryState.hpp"

namespace devils
{
    class Trajectory
    {
    public:
        /**
         * Creates a new instance of a trajectory
         */
        Trajectory(std::vector<TrajectoryState *> trajectoryStates)
        {
            this->trajectoryStates = trajectoryStates;
            this->duration = trajectoryStates.back()->time;
        }

        /**
         * Gets the trajectory state at a given time
         * @param t The time in seconds
         * @return The trajectory state
         */
        TrajectoryState getStateAt(double t)
        {
            // Check OOB
            if (t <= 0)
                return *trajectoryStates.front();
            if (t >= duration)
                return *trajectoryStates.back();

            // Binary search for the state
            auto nextStateItr = std::lower_bound(
                trajectoryStates.begin(),
                trajectoryStates.end(),
                t,
                [](TrajectoryState *state, double time)
                {
                    return state->time < time;
                });
            auto prevStateItr = nextStateItr - 1;

            // Get the states
            TrajectoryState *nextState = *nextStateItr;
            TrajectoryState *prevState = *prevStateItr;

            // Calculate dt between the two states
            // This is the percentage of the way between the two states
            double dt = (t - prevState->time) / (nextState->time - prevState->time);

            // Interpolate between the two states
            return prevState->interpolate(nextState, dt);
        }

        /**
         * Gets the duration of the trajectory
         * @return The duration in seconds
         */
        double getDuration()
        {
            return duration;
        }

    private:
        std::vector<TrajectoryState *> trajectoryStates;
        double duration;
    };
}