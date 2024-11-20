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
        /**
         * Generates a trajectory from a path.
         * Use `applyConstraints` to apply custom constraints to the generator.
         * @param path The path to generate the trajectory from
         * @return The generated trajectory
         */
        Trajectory generateTrajectory(Path *path)
        {
            // Initialize the trajectory
            double t = 0;
            std::vector<TrajectoryState *> trajectoryStates;
            trajectoryStates.reserve(path->getLength() / DELTA_T);

            // Set initial state
            TrajectoryState *prevState = new TrajectoryState();
            prevState->currentPose = path->getPoseAt(0);
            prevState->acceleration = constraints.maxAcceleration;
            prevState->velocity = 0; // TODO: Set initial velocity
            trajectoryStates.push_back(prevState);

            // Run Forward Pass
            for (double i = DELTA_T; i < path->getLength(); i += DELTA_T)
            {
                // Create new state
                TrajectoryState *state = new TrajectoryState();
                state->currentPose = path->getPoseAt(i);
                trajectoryStates.push_back(state);

                // Forward pass
                forwardPass(prevState, state);

                // Update previous state
                prevState = state;
            }

            // Run Backward Pass
            for (int i = trajectoryStates.size() - 2; i >= 0; i--)
            {
                // Create new state
                TrajectoryState *state = trajectoryStates[i];

                // Backward pass
                backwardPass(prevState, state);

                // Update previous state
                prevState = state;
            }

            return Trajectory(trajectoryStates);
        }

        /**
         * Applies constraints to the trajectory generator
         * @param constraints The constraints to apply
         */
        void applyConstraints(TrajectoryConstraints constraints)
        {
            // TODO: Implement additional constraints (Tank model, centrifugal acceleration, etc.)
            this->constraints = constraints;
        }

    private:
        /**
         * Runs a forward pass given a previous and current state.
         * States are modified in place to meet constraints.
         * @param prevState The previous state
         * @param state The current state
         */
        void forwardPass(
            TrajectoryState *prevState,
            TrajectoryState *state)
        {
            // Calculate distance between states
            double distance = state->currentPose.distanceTo(prevState->currentPose);

            // Contunuously iterate until we reach desired constraints
            while (true)
            {
                // Calculate current velocity from previous acceleration
                // v_f = sqrt(v_i^2 + 2 * a * d)
                state->velocity = sqrt(
                    prevState->velocity * prevState->velocity +
                    2 * prevState->acceleration * distance);

                // Constrain velocity
                state->velocity = std::min(state->velocity, constraints.maxVelocity);

                // Default to max acceleration
                // This will be modified in subsequent iterations
                state->acceleration = constraints.maxAcceleration;

                // TODO: Constain velocity here to other models
                // TODO: Constain acceleration here to other models

                // If we are within epsilon, assume we are done
                // This is to prevent division by zero or
                // errors caused by decimal precision
                if (distance < constraints.epsilon)
                    break;

                // Calculate actual acceleration between previous and current velocity
                // a = (v_f^2 - v_i^2) / (2 * d)
                double actualAcceleration = (state->velocity * state->velocity -
                                             prevState->velocity * prevState->velocity) /
                                            (2 * distance);

                // If the actual acceleration is within accel constraints, we're good
                bool isWithinConstaints = actualAcceleration - constraints.epsilon <= constraints.maxAcceleration;
                if (isWithinConstaints)
                    break;

                // Otherwise, reduce previous acceleration and try again
                prevState->acceleration = actualAcceleration;
            }
        }

        /**
         * Runs a backward pass given a previous and current state.
         * States are modified in place to meet constraints.
         * @param prevState The previous state
         * @param state The current state
         */
        void backwardPass(
            TrajectoryState *prevState,
            TrajectoryState *state)
        {
            // Calculate distance
            double distance = state->currentPose.distanceTo(prevState->currentPose);

            // Iterate until we reach desired constraints
            while (true)
            {
                // TODO: Implement backward pass
                break;
            }
        }

        constexpr static double DELTA_T = 0.01;

        TrajectoryConstraints constraints;
    };
}