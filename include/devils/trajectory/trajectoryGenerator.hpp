#pragma once

#include <vector>
#include <algorithm>
#include <stdexcept>
#include <cmath>
#include "trajectory.hpp"
#include "trajectoryConstraints.hpp"
#include "../path/path.hpp"
#include "../geometry/units.hpp"

namespace devils
{
    /**
     *  Generates a `Trajectory` from a `Path` using a trapezoidal motion profile.
     */
    class TrajectoryGenerator
    {
    public:
        /// @brief Path parameters
        struct PathInfo
        {
            /// @brief The initial velocity in inches per second
            double startingVelocity = 0;

            /// @brief The final velocity in inches per second
            double endingVelocity = 0;
        };

        /**
         * Creates a new trajectory generator.
         * @param constraints The robot constraints
         * @param pathInfo The path information
         */
        TrajectoryGenerator(TrajectoryConstraints constraints, PathInfo pathInfo)
            : constraints(constraints),
              pathInfo(pathInfo)
        {
        }

        /**
         * Calculates the trajectory from a path.
         * This is a resource-intensive operation and should be called sparingly.
         * @param path The path to generate the trajectory from
         * @return The generated trajectory
         */
        std::shared_ptr<Trajectory> calc(Path &path)
        {
            // The velocities at each step
            auto points = std::make_unique<std::vector<Trajectory::Point>>();
            points->reserve(path.getLength() / DELTA_INDEX + 1);

            // Initial Robot State
            Trajectory::Point previousPoint = {
                0,
                path.getPoseAt(0),
                pathInfo.startingVelocity,
                0,
                constraints.maxAcceleration};
            points->push_back(previousPoint);

            // FORWARD PASS
            // Iterate through the path and constrain the velocity in the acceleration phase
            for (double i = 0; i <= path.getLength() - 1 + DELTA_INDEX; i += DELTA_INDEX)
            {
                // Get pose
                Pose currentPose = path.getPoseAt(i);

                // Calculate distance
                double deltaDistance = currentPose.distanceTo(previousPoint.pose);

                // Calculate velocity
                // v_f = sqrt(v_i^2 + 2*a*d)
                double velocity = sqrt(pow(previousPoint.velocity, 2) + 2 * constraints.maxAcceleration * deltaDistance);

                // Clamp velocity
                velocity = std::min(velocity, constraints.maxVelocity);

                // Append to points
                Trajectory::Point point = {
                    0, // Time is calculated later
                    currentPose,
                    velocity,
                    0,
                    0};
                points->push_back(point);

                // Update previous point
                previousPoint = point;
            }

            // Ending Robot State
            previousPoint.velocity = pathInfo.endingVelocity;

            // REVERSE PASS
            // Iterate through the path and constrain the velocity in the deceleration phase
            for (double i = points->size() - 1; i >= 0; i--)
            {
                // Grab existing point
                auto &point = points->at(i);

                // Calculate distance
                double deltaDistance = point.pose.distanceTo(previousPoint.pose);

                // Calculate velocity
                // v_f = sqrt(v_i^2 + 2*a*d)
                double velocity = sqrt(pow(previousPoint.velocity, 2) + 2 * constraints.maxAcceleration * deltaDistance);

                // Clamp velocity to existing point
                velocity = std::min(velocity, point.velocity);

                // Update previous point
                point.velocity = velocity;
                previousPoint = point;
            }

            // Reset `previousPoint` to the first point
            previousPoint = points->front();

            // FINAL PASS
            // Calculate the time, acceleration, and angular velocity for each point
            for (int i = 1; i < points->size(); i++)
            {
                // Get points
                auto &point = points->at(i);

                // Calculate distance
                double deltaDistance = point.pose.distanceTo(previousPoint.pose);

                // Calculate acceleration
                // a = (v_f^2 - v_i^2) / (2 * d)
                double acceleration = (pow(point.velocity, 2) - pow(previousPoint.velocity, 2)) / (2 * deltaDistance);
                if (std::isnan(acceleration))
                    acceleration = 0;

                // Calculate delta time
                // 0 = (1/2)at^2 + v_i*t - d
                // t = (-v_i + sqrt(v_i^2 + 2ad)) / a
                double deltaTime = std::pow(previousPoint.velocity, 2) + 2 * acceleration * deltaDistance;
                deltaTime = (-previousPoint.velocity + std::sqrt(deltaTime)) / acceleration;

                // If the time is NaN, acceleration is 0
                // t = d / v
                if (std::isnan(deltaTime))
                    deltaTime = deltaDistance / previousPoint.velocity;

                // If the time is NaN, set to 0
                if (std::isnan(deltaTime))
                    deltaTime = 0;

                // Calculate angular velocity
                double angularVelocity = Units::diffRad(point.pose.rotation, previousPoint.pose.rotation) / deltaTime;
                if (std::isnan(angularVelocity))
                    angularVelocity = 0;

                // Update point
                point.t = previousPoint.t + deltaTime;
                point.acceleration = acceleration;

                // Update previous point
                previousPoint = point;
            }

            return std::make_shared<Trajectory>(std::move(points));
        }

    private:
        /// @brief The step size in path indices
        static constexpr double DELTA_INDEX = 0.05;

        /// @brief Constraints of trajectory generation
        TrajectoryConstraints constraints;

        /// @brief The path information
        PathInfo pathInfo;
    };
}