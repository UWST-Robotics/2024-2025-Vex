#pragma once

#include "pros/rtos.hpp"
#include "devils/utils/logger.hpp"

namespace devils
{
    class Timer
    {
    public:
        /**
         * Creates a new timer with a given duration.
         * @param duration The duration of the timer in milliseconds.
         */
        void start(uint32_t duration)
        {
            this->startTime = pros::millis();
            this->duration = duration;
            this->isStarted = true;
        }

        /**
         * Stops the timer.
         */
        void stop()
        {
            this->isStarted = false;
        }

        /**
         * Gets whether the timer has started.
         * @return True if the timer has started, false otherwise.
         */
        bool running()
        {
            return isStarted && pros::millis() - startTime < duration;
        }

        /**
         * Gets whether the timer has finished.
         * @return True if the timer has finished, false otherwise.
         */
        bool finished()
        {
            return isStarted && pros::millis() - startTime >= duration;
        }

    private:
        bool isStarted = false;
        uint32_t startTime = 0;
        uint32_t duration = 0;
    };
}