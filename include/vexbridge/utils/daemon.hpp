#pragma once

#include "pros/rtos.hpp"

namespace vexbridge::utils
{
    /**
     * Represents a daemon that runs in the background while the user program is running.
     * The method `update` is called repeatedly in a separate PROS task.
     * Be sure to call `pros::delay` in `update` to prevent the task from consuming too much CPU time.
     */
    class Daemon
    {
    public:
        Daemon() : daemonTask([=, this]
                              { runTask(); })
        {
        }

    protected:
        /**
         * Called repeatedly in a separate PROS task.
         * Be sure to call `pros::delay` in this method to prevent the task from consuming too much CPU time.
         */
        virtual void update() = 0;

    private:
        /**
         * The task method that runs the `update` method.
         */
        void runTask()
        {
            while (true)
            {
                try
                {
                    update();
                }
                catch (const std::exception &e)
                {
                    printf("Daemon exception: %s\n", e.what());
                    pros::delay(REVIVE_DELAY);
                }
            }
        }

        // Disable copy and assignment
        Daemon(const Daemon &) = delete;
        Daemon &operator=(const Daemon &) = delete;

        // Constants
        static constexpr uint32_t REVIVE_DELAY = 1000;

        // The task that runs the daemon
        const pros::Task daemonTask;
    };
};