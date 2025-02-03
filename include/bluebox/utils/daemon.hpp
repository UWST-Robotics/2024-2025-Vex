#pragma once

#include "pros/rtos.hpp"

namespace bluebox
{
    /**
     * Represents a daemon that runs in the background while the user program is running.
     * The method `update` is called repeatedly in a separate PROS task.
     * Be sure to call `pros::delay` in `update` to prevent the task from consuming too much CPU time.
     */
    class Daemon
    {
    public:
        Daemon() : daemonTask([=]
                              { runTask(); })
        {
        }
        ~Daemon()
        {
            daemonTask.remove();
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
                update();
        }

        Daemon(const Daemon &) = delete;
        Daemon &operator=(const Daemon &) = delete;

        pros::Task daemonTask;
    };
};