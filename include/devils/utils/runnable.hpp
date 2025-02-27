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
        ~Runnable() { stop(); }

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
         * Function that is called to check if the object is finished running.
         * @return True if the object is finished running.
         */
        virtual bool checkFinished() { return false; }

        /**
         * Runs the object asynchronously.
         * @return The PROS task that runs the object.
         */
        pros::Task *runAsync()
        {
            // Stop any existing async tasks
            stop();

            // Start task asynchronously
            currentTask = new pros::Task(
                [=, this]
                {
                    // Start Event
                    onStart();

                    // Loop
                    while (!checkFinished())
                    {
                        try
                        {
                            onUpdate();
                        }
                        catch (const std::exception &e)
                        {
                            Logger::error("An error occurred in Runnable: " + std::string(e.what()));
                        }
                        pros::delay(updateInterval);
                    }

                    // Stop Event
                    onStop();
                });
            return currentTask;
        }

        /**
         * Stops the object from running.
         * Deletes the task and calls onStop.
         */
        void stop()
        {
            if (currentTask != nullptr)
            {
                onStop();
                currentTask->remove();
                delete currentTask;
                currentTask = nullptr;
            }
        }

        /**
         * Runs the object synchronously.
         */
        void run()
        {
            runAsync();
            currentTask->join();
        }

    private:
        pros::Task *currentTask = nullptr;
        int updateInterval = 20;
    };
}