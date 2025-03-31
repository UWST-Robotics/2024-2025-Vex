#pragma once

#include "pros/rtos.hpp"
#include "devils/controller/controllerBase.h"

namespace devils
{
    /**
     * Represents a feedback controller that uses a PID algorithm.
     */
    class PIDController : public ControllerBase
    {
    public:
        /// @brief The options for the PID controller.
        struct Options
        {
            /// @brief Proportional gain (p * error)
            double pGain = 0.0;

            /// @brief Integral gain (i * integral)
            double iGain = 0.0;

            /// @brief Derivative gain (d * derivative)
            double dGain = 0.0;
        };

        /**
         * Creates a new PID controller.
         * @param pGain The proportional gain of the controller (p * error)
         * @param iGain The integral gain of the controller (i * integral)
         * @param dGain The derivative gain of the controller (d * derivative)
         */
        PIDController(
            double pGain,
            double iGain,
            double dGain)
            : pGain(pGain),
              iGain(iGain),
              dGain(dGain)
        {
        }

        /**
         * Creates a new PID controller with the given options.
         * @param options The options for the PID controller.
         */
        PIDController(Options options)
            : pGain(options.pGain),
              iGain(options.iGain),
              dGain(options.dGain)
        {
        }

        /**
         * Resets the PID controller to its initial state.
         * Clears all error and integral values.
         * Should be called before starting a new control loop.
         */
        void reset()
        {
            currentError = 0;
            currentIntegral = 0;
            currentDerivative = 0;
            lastError = 0;
            lastUpdateTimestamp = pros::millis();
        }

        /**
         * Updates the PID controller with a new error value.
         * Should be called every control loop iteration.
         * @param currentError The current error value.
         * @return The current output value of the PID controller.
         */
        double update(double currentError) override
        {
            // Get Delta Time
            double dt = pros::millis() - lastUpdateTimestamp;
            lastUpdateTimestamp = pros::millis();

            // Update Error
            this->currentError = currentError;

            // Update Integral
            currentIntegral += currentError * dt;

            // Update Derivative
            if (dt > 0)
                currentDerivative = (currentError - lastError) / dt;
            lastError = currentError;

            // Return Value
            return getValue();
        }

        /**
         * Gets the last output value of the PID controller without updating it.
         * @return The last output value of the PID controller.
         */
        double getValue()
        {
            double p = pGain * currentError;
            double i = iGain * currentIntegral;
            double d = dGain * currentDerivative;

            return p + i + d;
        }

    private:
        // Feedback
        double currentError = 0;
        double currentIntegral = 0;
        double currentDerivative = 0;

        // Last Values
        double lastError = 0;
        double lastUpdateTimestamp = 0;

        // PID Variables
        double pGain;
        double iGain;
        double dGain;
    };

}