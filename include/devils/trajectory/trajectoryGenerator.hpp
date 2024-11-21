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
            std::vector<TrajectoryState *> trajectoryStates;
            trajectoryStates.reserve(path->getLength() / DELTA_T);

            // Set initial state
            TrajectoryState *prevState = new TrajectoryState();
            prevState->currentPose = path->getPoseAt(0);
            prevState->acceleration = constraints.maxAcceleration;
            prevState->velocity = constraints.initialVelocity;
            trajectoryStates.push_back(prevState);

            // Run Forward Pass
            for (double i = DELTA_T; i < path->getLength(); i += DELTA_T)
            {
                // Create new state
                TrajectoryState *state = new TrajectoryState();
                state->currentPose = path->getPoseAt(i);
                state->time = i;
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
                TrajectoryState *nextState = trajectoryStates[i + 1];

                // Backward pass
                backwardPass(state, nextState);
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
         * @param nextState The next state
         */
        void forwardPass(
            TrajectoryState *prevState,
            TrajectoryState *nextState)
        {
            // Calculate distance between states
            double distance = nextState->currentPose.distanceTo(prevState->currentPose);

            // TODO: Implement forward pass
        }

        /**
         * Runs a backward pass given a previous and current state.
         * States are modified in place to meet constraints.
         * @param prevState The previous state
         * @param nextState The next state
         */
        void backwardPass(
            TrajectoryState *prevState,
            TrajectoryState *nextState)
        {
            // Calculate distance between states
            double distance = nextState->currentPose.distanceTo(prevState->currentPose);

            // TODO: Implement backward pass
        }

        constexpr static double DELTA_T = 0.01;

        TrajectoryConstraints constraints;
    };
}