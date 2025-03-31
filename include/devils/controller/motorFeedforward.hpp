#pragma once
#include <cmath>
#include <algorithm>

namespace devils
{
    /**
     * Represents a feedforward controller for a motor.
     * Assumes no gravity or external forces act on the motor (See `ArmFeedforward` or `ElevatorFeedforward` for those cases).
     */
    class MotorFeedforward
    {
    public:
        struct Options
        {
            /// @brief Amount of force required to overcome static friction.
            double staticFriction = 0.0;

            /// @brief Amount of voltage to apply a given velocity.
            double velocityGain = 0.0;

            /// @brief Amount of voltage to apply a given acceleration.
            double accelerationGain = 0.0;
        };

        MotorFeedforward(Options options)
            : options(options)
        {
        }

        /**
         * Updates the feedforward voltage.
         * @param velocity The desired velocity of the motor.
         * @param acceleration The desired acceleration of the motor.
         * @return The voltage to apply to the motor.
         */
        double update(
            double velocity = 0,
            double acceleration = 0)
        {
            // V = output voltage
            // k_s = static friction
            // k_v = velocity gain
            // k_a = acceleration gain
            // V = k_g + k_s * sign(vel) + k_v * vel + k_a * acc

            double voltage = 0.0;
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