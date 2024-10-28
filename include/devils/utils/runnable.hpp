#pragma once
#include "pros/rtos.hpp"

namespace devils
{
    /**
     * Represents a runnable object w/ an update loop.
     */
    class Runnable
    {
    public:
        Runnable() = default;
        Runnable(int updateInterval) : updateInterval(updateInterval) {}

        /**
         * Update function that is called periodically.
         */
        virtual void update() = 0;

        /**
         * Runs the object synchronously.
         */
        virtual void runSync()
        {
            while (true)
            {
                update();
                pros::delay(updateInterval);
            }
        }

        /**
         * Runs the object asynchronously.
         * @return The PROS task that runs the object.
         */
        virtual pros::Task runAsync()
        {
            return pros::Task([=, this]
                              { runSync(); });
        }

    private:
        int updateInterval = 20;
    };

    /**
     * Represents a runnable object that runs automatically when created.
     */
    struct AutoRunnable : public Runnable
    {
        pros::Task runTask;

        /**
         * Automatically runs the object asynchronously.
         */
        AutoRunnable() : runTask(runAsync())
        {
        }
    };
}