#pragma once
#include "pros/rtos.hpp"
#include "../utils/logger.hpp"
#include <ctime>

namespace devils
{
    /**
     * Represents a simple PID controller.
     */
    class PID
    {
    public:
        /**
         * Creates a new PID controller.
         * @param pGain The proportional constant.
         * @param iGain The integral constant.
         * @param dGain The derivative constant.
         */
        PID(double pGain,
            double iGain,
            double dGain,
            double maxIntegral = 1)
            : pGain(pGain),
              iGain(iGain),
              dGain(dGain),
              maxIntegral(maxIntegral),
              maxError(maxIntegral / iGain)
        {
        }

        double update(double actual, double setpoint = 0)
        {
            double error = setpoint - actual;

            // Proportional
            double p = pGain * error;

            // Integral
            double i = iGain * errorSum;
            i = std::clamp(i, -maxIntegral, maxIntegral);

            // Derivative
            double d = -dGain * (actual - lastActual);

            // Error
            errorSum += error;
            errorSum = std::clamp(errorSum, -maxError, maxError);

            // Output
            double output = p + i + d;
            output = std::clamp(output, -maxOutput, maxOutput);

            // Save Last Values
            lastActual = actual;

            // Debug
            // Logger::debug("P=" + std::to_string(p) + " I=" + std::to_string(i) + " D=" + std::to_string(d) + " O=" + std::to_string(output) + " A=" + std::to_string(actual) + " S=" + std::to_string(setpoint));

            return output;
        }

        /**
         * Sets the maximum output of the PID controller.
         * @param maxOutput The maximum output of the PID controller.
         */
        void setMaxOutput(double maxOutput)
        {
            this->maxOutput = maxOutput;
        }

        void reset()
        {
            errorSum = 0;
            lastActual = 0;
        }

    private:
        const double pGain;
        const double iGain;
        const double dGain;
        const double maxIntegral = 1;
        const double maxError = 1;

        double setpoint = 0;
        double errorSum = 0;
        double lastActual = 0;
        double maxOutput = 1;
    };
}