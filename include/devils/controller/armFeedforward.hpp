#pragma once
#include <cmath>
#include <algorithm>

namespace devils
{
    /**
     * Represents a feedforward controller for a single-jointed arm.
     * Assumes gravity acts on the arm proportional to the angle of the arm.
     */
    class ArmFeedforward
    {
    public:
        struct Options
        {
            /// @brief Amount of gravitational force applied to the arm.
            double gravity = 0.0;

            /// @brief Amount of force required to overcome static friction.
            double staticFriction = 0.0;

            /// @brief Amount of voltage to apply a given velocity.
            double velocityGain = 0.0;

            /// @brief Amount of voltage to apply a given acceleration.
            double accelerationGain = 0.0;
        };

        ArmFeedforward(Options options)
            : options(options)
        {
        }

        /**
         * Updates the feedforward voltage.
         * @param position The position of the arm in radians.
         * @param velocity The desired velocity of the arm.
         * @param acceleration The desired acceleration of the arm.
         * @return The voltage to apply to the motor.
         */
        double update(
            double position = 0,
            double velocity = 0,
            double acceleration = 0)
        {
            // V = output voltage
            // k_g = gravity
            // k_s = static friction
            // k_v = velocity gain
            // k_a = acceleration gain
            // pos = position
            // V = k_g * cos(pos) + k_s * sign(vel) + k_v * vel + k_a * acc

            double voltage = 0.0;
            voltage += options.gravity * std::cos(position);                // Gravity
            voltage += options.staticFriction * std::copysign(1, velocity); // Static friction
            voltage += options.velocityGain * velocity;                     // Velocity
            voltage += options.accelerationGain * acceleration;             // Acceleration

            // Clamp the voltage to the range [-1, 1]
            voltage = std::clamp(voltage, -1.0, 1.0);

            return voltage;
        }

    private:
        Options options;
    };
}