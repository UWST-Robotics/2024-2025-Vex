#pragma once

namespace devils
{
    /**
     * Represents various constraints for generating a trajectory.
     * Defines how fast the robot can move and accelerate over time.
     */
    struct TrajectoryConstraints
    {
        /// @brief The maximum velocity of the robot in inches per second
        double maxVelocity = 36; // in/s

        /// @brief The maximum acceleration of the robot in inches per second squared
        double maxAcceleration = 36; // in/s^2
    };
}