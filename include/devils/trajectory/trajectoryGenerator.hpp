#pragma once

#include <vector>
#include "../geometry/pose.hpp"
#include "structs/trajectoryConstraints.hpp"
#include "trajectory.hpp"
#include "../path/path.hpp"

namespace devils
{
    class TrajectoryGenerator
    {
    public:
        TrajectoryGenerator() = delete;

        /**
         * Generates a trajectory from a path
         * @param path The path to generate the trajectory from
         * @param constraints The constraints for the trajectory
         * @return The generated trajectory
         */
        static Trajectory generateTrajectory(
            Path *path,
            TrajectoryConstraints constraints)
        {
            // Initialize the trajectory
            double t = 0;
            std::vector<TrajectoryState *> trajectoryStates;

            // Set initial state
            TrajectoryState *prevState = new TrajectoryState();
            prevState->currentPose = path->getPoseAt(0);

            // Step through path
            for (double i = 0; i < path->getLength(); i += DELTA_T)
            {
                // Get the pose at the current index
                Pose currentPose = path->getPoseAt(i);

                // Calculate jerk
                double jerk = (currentPose.rotation - prevState->currentPose.rotation) / DELTA_T;
                jerk = std::clamp(jerk, -constraints.maxJerk, constraints.maxJerk);

                // Calculate acceleration
                double acceleration = (jerk * DELTA_T) + prevState->acceleration;
                acceleration = std::clamp(acceleration, -constraints.maxAcceleration, constraints.maxAcceleration);

                // Calculate velocity
                double velocity = (acceleration * DELTA_T) + prevState->velocity;
                velocity = std::clamp(velocity, -constraints.maxVelocity, constraints.maxVelocity);
            }

            return Trajectory(trajectoryStates);
        }

    private:
        constexpr static double DELTA_T = 0.01;
    };
}