#pragma once

namespace devils
{
    /**
     * Represents various constraints for trajectory limitations.
     * Used to ensure that the robot does not exceed certain limits.
     */
    struct TrajectoryConstraints
    {
        /// @brief The maximum velocity of the robot in inches per second
        double maxVelocity;

        /// @brief The maximum acceleration of the robot in inches per second squared
        double maxAcceleration;

        /// @brief The maximum jerk of the robot in inches per second cubed
        double maxJerk;
    };
}