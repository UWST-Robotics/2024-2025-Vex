#pragma once

#include "motionProfile.hpp"
#include <vector>
#include <algorithm>
#include <stdexcept>

namespace devils
{
    /**
     *  Represents a 1-dimensional S-Curve motion profile
     */
    class SCurveMotionProfile : public MotionProfile
    {
    public:
        struct Options
        {
            /// @brief The starting velocity in inches per second
            double startVelocity;

            /// @brief The ending velocity in inches per second
            double endVelocity;

            /// @brief The maximum velocity in inches per second
            double maxVelocity;

            /// @brief The maximum acceleration in inches per second squared
            double maxAcceleration;
        };

        struct ProfilePoint
        {
            double time;
            double distance;
            double maxVelocity;
        };

        /**
         * Creates a new S-Curve motion profile
         * @param goalDistance The goal distance in inches
         */
        SCurveMotionProfile(double goalDistance)
        {
            auto accelerationProfile = calc(options.startVelocity, goalDistance);
            auto decelerationProfile = calc(options.endVelocity, goalDistance);

            // Merge the two profiles
            for (auto &point : accelerationProfile)
                allVelocities.push_back(point);

            for (int i = decelerationProfile.size() - 1; i >= 0; i--)
            {
                auto &point = decelerationProfile[i];
                allVelocities.push_back(point);
            }
        }

        std::vector<ProfilePoint> calc(
            double startVelocity,
            double goalDistance)
        {
            // The velocities at each step
            std::vector<ProfilePoint> velocities;

            // TODO: Implement jerk
            double currentAcceleration = options.maxAcceleration;
            double currentVelocity = startVelocity;
            double currentPosition = 0;

            // Current time
            double time = 0;

            // Step forward from 0 to goal distance
            while (currentPosition < goalDistance)
            {
                // Calculate velocity
                currentVelocity += currentAcceleration * deltaT;

                // Check if we reached max velocity
                if (currentVelocity > options.maxVelocity)
                    break;

                // Append to velocities
                velocities.push_back(ProfilePoint{time, currentPosition, currentVelocity});

                // Update current distance
                currentPosition += currentVelocity * deltaT;

                // Update time
                time += deltaT;

                // Abort if we exceed max time
                if (time > maxTime)
                    throw std::runtime_error("Exceeded max time calculating motion profile");
            }

            return velocities;
        }

        double getSpeed(double distanceToGoal) override
        {
            // Iterate through the velocities
            for (auto &point : allVelocities)
            {
                // Find the velocity at the distance to the goal
                if (point.distance >= distanceToGoal)
                    return point.maxVelocity;
            }

            return options.maxVelocity;
        }

    private:
        /// @brief The step size in seconds
        static constexpr double deltaT = 0.05;

        /// @brief The maximum allowed time in seconds
        static constexpr double maxTime = 20.0;

        /// @brief The velocities at each step
        std::vector<ProfilePoint> allVelocities;

        /// @brief The options for the motion profile
        Options options;
    };
}