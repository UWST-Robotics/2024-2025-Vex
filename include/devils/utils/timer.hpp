#pragma once

#include "pros/rtos.hpp"
#include "devils/utils/logger.hpp"

namespace devils
{
    class Timer
    {
    public:
        /**
         * Creates a new instance of Timer with a given duration.
         * @param duration The duration of the timer in milliseconds.
         * @return The new instance of Timer.
         */
        Timer(uint32_t duration) : duration(duration)
        {
        }

        /**
         * Sets the duration of the timer.
         * @param duration The duration of the timer in milliseconds.
         */
        void setDuration(uint32_t duration)
        {
            this->duration = duration;
        }

        /**
         * Starts the timer.
         */
        void start()
        {
            this->startTime = pros::millis();
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

        /**
         * Gets the time remaining on the timer.
         * @return The time remaining on the timer in milliseconds.
         */
        double timeRemaining()
        {
            return duration - (pros::millis() - startTime);
        }

    private:
        bool isStarted = false;
        uint32_t startTime = 0;
        uint32_t duration = 0;
    };
}