#pragma once

#include <vector>
#include <algorithm>
#include <stdexcept>
#include <cmath>
#include "trajectoryConstraints.hpp"

namespace devils
{
    /**
     *  Represents a 1-dimensional trapezoidal motion profile
     */
    class TrapezoidMotionProfile
    {
    public:
        /// @brief Path parameters
        struct PathInfo
        {
            /// @brief The initial velocity in inches per second
            double startingVelocity;

            /// @brief The final velocity in inches per second
            double endingVelocity;

            /// @brief The distance to the goal in inches
            double goalDistance;
        };

        /// @brief Point along the profile
        struct ProfilePoint
        {
            /// @brief The position in inches
            double position;

            /// @brief The velocity in inches per second
            double velocity;
        };

        /**
         * Creates a new trapezoidal motion profile.
         * Make sure to call `calc()` to calculate the profile before using it.
         * @param constraints The robot constraints
         * @param pathInfo The path information
         */
        TrapezoidMotionProfile(TrajectoryConstraints constraints, PathInfo pathInfo)
            : constraints(constraints),
              pathInfo(pathInfo)
        {
        }

        /**
         * Calculates the motion profile.
         * This is a resource-intensive operation and should be called sparingly.
         */
        void calc()
        {
            // Check if we've already calculated the profile
            if (accelVelocities.size() > 0 && decelVelocities.size() > 0)
                return;

            // Calculate acceleration phase
            // This will create the acceleration segment of the S-Curve
            accelVelocities = calculateAccelVelocities(pathInfo.startingVelocity);

            // Calculate deceleration phase
            // This will create the deceleration segment of the S-Curve
            decelVelocities = calculateAccelVelocities(pathInfo.endingVelocity);
        }

        double getSpeed(double position)
        {
            // Get each velocity
            double accelVelocity = getVelocityAtPosition(accelVelocities, position);
            double decelVelocity = getVelocityAtPosition(decelVelocities, pathInfo.goalDistance - position);

            // Clamp to the minimum of the two curves
            return std::min(accelVelocity, decelVelocity);
        }

    protected:
        /**
         * Steps through the list of profile points and returns the velocity at the given position.
         * Lerps between the two closest points.
         * @param velocities The list of profile points
         * @param position The current position in inches
         * @return The target velocity in inches per second
         */
        double getVelocityAtPosition(std::vector<ProfilePoint> &velocities, double position)
        {
            // Check if we've calculated the profile
            if (velocities.size() == 0)
                throw std::runtime_error("No velocities calculated. Call `calc()` first.");

            // Iterate through the list of velocities
            // TODO: Replace with binary search
            for (size_t i = 1; i < velocities.size(); i++)
            {
                // Check if we've passed the position
                if (velocities[i].position >= position)
                {
                    // Get the two closest points
                    auto &prev = velocities[i - 1];
                    auto &next = velocities[i];

                    // Lerp position
                    double t = (position - prev.position) / (next.position - prev.position);
                    t = std::clamp(t, 0.0, 1.0);

                    // Lerp velocity
                    return std::lerp(prev.velocity, next.velocity, t);
                }
            }

            // Fallback to the last point
            return velocities.back().velocity;
        }

        /**
         * Calculates a list of profile points by accelerating from `startingVelocity` to the goal distance.
         * @param startingVelocity The initial velocity in inches per second
         * @return The list of profile points sequential order
         */
        std::vector<ProfilePoint> calculateAccelVelocities(double startingVelocity)
        {
            // The velocities at each step
            std::vector<ProfilePoint> velocities;

            // Current Robot State
            double currentAcceleration = constraints.maxAcceleration;
            double currentVelocity = startingVelocity;
            double currentPosition = 0;
            double currentTime = 0;

            // Step forward from 0 to goal distance
            while (currentPosition < pathInfo.goalDistance)
            {
                // Calculate velocity
                currentVelocity += currentAcceleration * DELTA_T;
                currentVelocity = std::min(currentVelocity, constraints.maxVelocity); // Constrain velocity

                // Update current distance
                currentPosition += currentVelocity * DELTA_T;

                // Update time
                currentTime += DELTA_T;

                // Append to velocities
                velocities.push_back(ProfilePoint{currentPosition, currentVelocity});

                // Abort if we reach max velocity
                // This saves processing time by ignoring vestigial points
                if (currentVelocity >= constraints.maxVelocity)
                    break;

                // Abort if we exceed max time
                // This is a safety check to prevent infinite loops
                if (currentTime > MAX_TIME)
                    throw std::runtime_error("Exceeded max timespan calculating motion profile. Double-check constraints.");
            }

            return velocities;
        }

    private:
        /// @brief The step size in seconds
        static constexpr double DELTA_T = 0.1;

        /// @brief The step size in inches
        static constexpr double DELTA_POS = 0.1;

        /// @brief The maximum allowed time in seconds
        static constexpr double MAX_TIME = 20.0;

        /// @brief The velocities along the acceleration curve
        std::vector<ProfilePoint> accelVelocities;

        /// @brief The velocities along the deceleration curve
        std::vector<ProfilePoint> decelVelocities;

        /// @brief Constraints of trajectory generation
        TrajectoryConstraints constraints;

        /// @brief The path information
        PathInfo pathInfo;
    };
}