#pragma once

#include <vector>
#include <memory>
#include "../geometry/pose.hpp"

namespace devils
{
    class Trajectory
    {
    public:
        /// @brief Represents the instantaneous state of a trajectory at a given time
        struct State
        {
            /// @brief The time in seconds
            double time = 0;

            /// @brief The linear velocity in inches per second
            double velocity = 0;

            /// @brief The angular velocity in radians per second
            double angularVelocity = 0;

            /// @brief The linear acceleration in inches per second squared
            double acceleration = 0;

            /// @brief The angular acceleration in radians per second squared
            double angularAcceleration = 0;

            /// @brief The current pose
            Pose pose = Pose(0, 0, 0);
        };

        /**
         * Creates a new instance of a trajectory
         * @param trajectoryStates A list of states along the trajectory
         */
        Trajectory(std::unique_ptr<std::vector<State>> trajectoryStates)
            : trajectoryStates(std::move(trajectoryStates))
        {
        }

        /**
         * Gets the duration of the trajectory in milliseconds
         */
        uint32_t duration() const
        {
            return trajectoryStates->back().time * 1000;
        }

        /**
         * Gets the trajectory state at a given time.
         * Lerps between the two closest calculated states.
         * @param t The time in seconds
         * @return The trajectory state at the given time
         */
        State getStateAt(const double t)
        {
            // Check if the trajectory is empty
            if (!trajectoryStates || trajectoryStates->empty())
                throw std::runtime_error("Trajectory is empty");

            // Check if the time is before the trajectory starts
            if (t <= 0)
                return trajectoryStates->front();

            // Find the two closest states to the given time
            for (size_t i = 0; i < trajectoryStates->size() - 1; i++)
            {
                // Get the two states
                auto previousState = trajectoryStates->at(i);
                auto nextState = trajectoryStates->at(i + 1);

                // Check if the time is between the two states
                if (t >= previousState.time && t <= nextState.time)
                {
                    // Calculate the interpolation time
                    double deltaTime = nextState.time - previousState.time;
                    double timeBetween = t - previousState.time;
                    double interpolationTime = timeBetween / deltaTime;

                    // Interpolate between the two states
                    return lerpStates(previousState, nextState, interpolationTime);
                }
            }

            // Default to the final state
            return trajectoryStates->back();
        }

    protected:
        /**
         * Linearly interpolates between two states.
         * @param a The first state
         * @param b The second state
         * @param t The time to interpolate (0 to 1)
         * @return The interpolated state
         */
        State lerpStates(const State a, const State b, const double t) const
        {
            State state;
            state.time = t;
            state.velocity = std::lerp(a.velocity, b.velocity, t);
            state.angularVelocity = std::lerp(a.angularVelocity, b.angularVelocity, t);
            state.acceleration = std::lerp(a.acceleration, b.acceleration, t);
            state.angularAcceleration = std::lerp(a.angularAcceleration, b.angularAcceleration, t);
            state.pose = Pose::lerp(a.pose, b.pose, t);
            return state;
        }

    private:
        std::unique_ptr<std::vector<State>> trajectoryStates;
    };
}