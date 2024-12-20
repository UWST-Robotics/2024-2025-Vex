#pragma once

namespace devils
{
    /**
     * Represents various constraints for trajectory limitations.
     * Used to ensure that the robot does not exceed certain limits.
     */
    struct TrajectoryConstraints
    {
        /// @brief The starting velocity in inches per second
        double initialVelocity = 0;

        /// @brief The maximum velocity of the robot in inches per second
        double maxVelocity = 36; // in/s

        /// @brief The maximum acceleration of the robot in inches per second squared
        double maxAcceleration = 36; // in/s^2

        /// @brief The maximum deceleration of the robot in inches per second squared
        double maxDeceleration = 36; // in/s^2

        /// @brief The maximum jerk of the robot in inches per second cubed
        double maxJerk = 36; // in/s^3

        /// @brief The maximum angular velocity of the robot in radians per second
        double maxAngularVelocity = 8; // rad/s

        /// @brief Values within this distance are considered equal
        double epsilon = 0.0001;
    };
}