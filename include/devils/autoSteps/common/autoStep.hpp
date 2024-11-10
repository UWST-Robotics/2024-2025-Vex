#pragma once
#include <string>

namespace devils
{
    /**
     * Represents a step in an autonomous routine.
     */
    struct IAutoStep
    {
        /**
         * Executes the step as a pros task.
         */
        virtual void doStep() = 0;
    };
}