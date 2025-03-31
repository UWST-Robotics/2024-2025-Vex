#pragma once

#include <cmath>
#include <algorithm>

namespace devils
{
    /**
     * Represents a SISO (Single Input Single Output) controller base.
     * Can be extended to provide feedforward and/or feedback control.
     */
    struct ControllerBase
    {
        /**
         * Updates the controller with a new input value.
         * @param input The new input value.
         * @return The output value of the controller.
         */
        virtual double update(double input) = 0;
    };
}