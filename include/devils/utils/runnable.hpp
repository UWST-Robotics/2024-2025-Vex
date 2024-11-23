#pragma once
#include "pros/rtos.hpp"

namespace devils
{
    /**
     * Represents an asynchronous object that can be run on a separate task.
     * Tasks are updated periodically at a given interval.
     */
    class Runnable
    {
    public:
        Runnable() = default;
        Runnable(int updateInterval) : updateInterval(updateInterval) {}
        ~Runnable() { stopAsync(); }

        /**
         * Function that is called when the object starts running.
         */
        virtual void onStart() {}

        /**
         * Function that is called when the object updates periodically.
         */
        virtual void onUpdate() {};

        /**
         * Function that is called when the object stops running.
         */
        virtual void onStop() {};

        /**
         * Runs the object asynchronously.
         * @return The PROS task that runs the object.
         */
        pros::Task *runAsync()
        {
            stopAsync();
            currentTask = new pros::Task(
                [=, this]
                {
                    onStart();
                    while (true)
                    {
                        try
                        {
                            onUpdate();
                        }
                        catch (const std::exception &e)
                        {
                            Logger::error("An error occurred in a Runnable task: " + std::string(e.what()));
                        }
                        pros::delay(updateInterval);
                    }
                });
            return currentTask;
        }

        /**
         * Stops the object from running.
         * Deletes the task and calls onStop.
         */
        void stopAsync()
        {
            if (currentTask != nullptr)
            {
                onStop();
                currentTask->remove();
                delete currentTask;
                currentTask = nullptr;
            }
        }

    private:
        pros::Task *currentTask = nullptr;
        int updateInterval = 20;
    };
}