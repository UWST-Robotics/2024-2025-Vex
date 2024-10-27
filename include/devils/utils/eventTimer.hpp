#pragma once

#include "pros/rtos.hpp"
#include "logger.hpp"

namespace devils
{
    class EventTimer
    {
    public:
        /**
         * Starts the timer if the ID is not already running.
         * @param timerID The ID of the timer or event.
         * @param duration The duration of the timer in milliseconds.
         */
        void start(int timerID, double duration)
        {
            if (lastTimerID == timerID)
                return;
            Logger::info("Starting Timer " + std::to_string(timerID) + "(from " + std::to_string(lastTimerID) + ")");

            lastTimerID = timerID;
            isRunning = true;
            startTime = pros::millis();
            this->duration = duration;
        }

        /**
         * Stops the timer.
         */
        void stop()
        {
            isRunning = false;
        }

        /**
         * Returns the time remaining on the timer in milliseconds.
         * @return The time remaining on the timer in milliseconds.
         */
        double getTimeRemaining()
        {
            if (!getRunning())
                return 0;
            return duration - (pros::millis() - startTime);
        }

        /**
         * Returns true if the timer is running.
         */
        bool getRunning()
        {
            if (isRunning && pros::millis() - startTime > duration)
                isRunning = false;
            return isRunning;
        }

    private:
        int lastTimerID = 0;
        bool isRunning = false;
        double startTime = 0;
        double duration = 0;
    };
}