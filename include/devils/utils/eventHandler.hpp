#pragma once

#include "logger.hpp"

namespace devils
{
    struct EventHandler
    {
        /**
         * Runs a local event with name and parameters.
         * This should run as a synchronous PROS task with `pros::delay()` or other call to allow task scheduling.
         * @param eventName The name of the event to run.
         * @param params The parameters to pass to the event.
         */
        virtual void runEvent(std::string eventName, std::string params[]) = 0;
    };
}