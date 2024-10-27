#pragma once
#include "pros/rtos.hpp"
#include "../utils/logger.hpp"

namespace devils
{
    /**
     * Represents a value that can be ramped up or down using a rate.
     */
    class Ramp
    {
    public:
        /**
         * Creates a new ramp.
         * @param maxPerSecond The maximum change in the ramp per second.
         * @param current The default value of the ramp.
         */
        Ramp(double maxPerSecond)
            : lastTimestamp(1),
              maxPerSecond(maxPerSecond),
              current(0)
        {
        }

        /**
         * Updates the ramp to the target value.
         * @param target The target value to ramp to.
         * @return The current value of the ramp.
         */
        double update(double target)
        {
            // Update Timestamp
            uint32_t timestamp = pros::millis();
            double delta = (double)(timestamp - lastTimestamp) / 1000.0;
            lastTimestamp = timestamp;

            // Update Current
            if (target > current)
                current = std::min(target, current + maxPerSecond * delta);
            else if (target < current)
                current = std::max(target, current - maxPerSecond * delta);
            return current;
        }

        /**
         * Sets the ramp rate of the ramp.
         * @param maxPerSecond The ramp rate of the ramp.
         */
        void setRampRate(double maxPerSecond)
        {
            this->maxPerSecond = maxPerSecond;
        }

    private:
        uint32_t lastTimestamp;
        double maxPerSecond;
        double current;
    };
}