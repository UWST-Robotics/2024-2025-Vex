#pragma once

#include "../../geometry/pose.hpp"

namespace devils
{
    /**
     * Represents the state of a robot trajectory at a given time.
     */
    struct TrajectoryState
    {
        /// @brief The time in seconds
        double time = 0;

        /// @brief The velocity in inches per second
        double velocity = 0;

        /// @brief The acceleration in inches per second squared
        double acceleration = 0;

        /// @brief The jerk in inches per second cubed
        double jerk = 0;

        /// @brief The current pose
        Pose currentPose = Pose(0, 0, 0);

        /**
         * Linearly interpolates between two trajectory states.
         * Starts with the current state and interpolates to the end state.
         * @param endState The end state
         * @param t The time to interpolate to. 0 is the current state, 1 is the end state.
         * @return The interpolated state
         */
        TrajectoryState lerp(TrajectoryState *endState, double t)
        {
            TrajectoryState interpolatedState;

            // Linearly interpolate the state
            interpolatedState.time = std::lerp(time, endState->time, t);
            interpolatedState.velocity = std::lerp(velocity, endState->velocity, t);
            interpolatedState.acceleration = std::lerp(acceleration, endState->acceleration, t);
            interpolatedState.jerk = std::lerp(jerk, endState->jerk, t);
            interpolatedState.currentPose = currentPose.lerp(endState->currentPose, t);

            return interpolatedState;
        }
    };
}