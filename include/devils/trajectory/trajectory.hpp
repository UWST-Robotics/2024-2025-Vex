#pragma once

#include <vector>
#include <memory>
#include "../geometry/pose.hpp"

namespace devils
{
    class Trajectory
    {
    public:
        /// @brief Point along the trajectory
        struct Point
        {
            /// @brief The current time in seconds
            double t;

            /// @brief The current position
            Pose pose;

            /// @brief The translation velocity in inches per second
            double velocity;

            /// @brief The angular acceleration in radians per second squared
            double angularVelocity;

            /// @brief The linear acceleration in inches per second squared
            double acceleration;
        };

        /**
         * Creates a new instance of a trajectory
         * @param trajectoryPoints A list of generated points along the trajectory
         */
        Trajectory(std::unique_ptr<std::vector<Point>> trajectoryPoints)
            : trajectoryPoints(std::move(trajectoryPoints))
        {
        }

        /**
         * Gets the duration of the trajectory in seconds
         * @return The duration of the trajectory in seconds
         */
        double duration() const
        {
            return trajectoryPoints->back().t;
        }

        /**
         * Gets the trajectory state at a given time.
         * Lerps between the two closest calculated states.
         * @param t The time in seconds
         * @return The trajectory state at the given time
         */
        Point getStateAt(const double t)
        {
            // Check if the trajectory is empty
            if (!trajectoryPoints || trajectoryPoints->empty())
                throw std::runtime_error("Trajectory is empty");

            // Check if the time is before the trajectory starts
            if (t <= 0)
                return trajectoryPoints->front();

            // Find the two closest states to the given time
            for (size_t i = 0; i < trajectoryPoints->size() - 1; i++)
            {
                // Get the two states
                auto previousState = trajectoryPoints->at(i);
                auto nextState = trajectoryPoints->at(i + 1);

                // Check if the time is between the two states
                if (t >= previousState.t && t <= nextState.t)
                {
                    // Calculate the interpolation time
                    double deltaTime = nextState.t - previousState.t;
                    double timeBetween = t - previousState.t;
                    double interpolationTime = timeBetween / deltaTime;

                    // Interpolate between the two states
                    return lerpStates(previousState, nextState, interpolationTime);
                }
            }

            // Default to the final state
            return trajectoryPoints->back();
        }

    protected:
        /**
         * Linearly interpolates between two states.
         * @param a The first state
         * @param b The second state
         * @param t The time to interpolate (0 to 1)
         * @return The interpolated state
         */
        Point lerpStates(const Point a, const Point b, const double t) const
        {
            Point state;
            state.t = t;
            state.pose = Pose::lerp(a.pose, b.pose, t);
            state.velocity = std::lerp(a.velocity, b.velocity, t);
            state.angularVelocity = std::lerp(a.angularVelocity, b.angularVelocity, t);
            state.acceleration = std::lerp(a.acceleration, b.acceleration, t);
            return state;
        }

    private:
        std::unique_ptr<std::vector<Point>> trajectoryPoints;
    };
}