#pragma once

#include "pros/rtos.hpp"

namespace devils
{
    /**
     * Represents the parameters of a PID controller.
     */
    struct PIDParams
    {
        double p;
        double i;
        double d;
    };

    /**
     * Represents a feedback controller that uses a PID algorithm.
     */
    class PIDController
    {
    public:
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

        PIDController(PIDParams params)
            : pGain(params.p),
              iGain(params.i),
              dGain(params.d)
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
        double update(double currentError)
        {
            // Logger::info("{E=" + std::to_string(currentError) + ", P=" + std::to_string(currentError) + ", I=" + std::to_string(currentIntegral) + ", D=" + std::to_string(currentDerivative) + "}");

            // Don't update if the error is the same
            // if (currentError == this->currentError)
            //     return getValue();

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
         * Gets the current output value of the PID controller.
         * @return The current output value of the PID controller.
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