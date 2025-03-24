#pragma once

namespace devils
{
    /**
     *  Represents a 1-dimensional motion profile
     */
    struct MotionProfile
    {
        /**
         * Gets the speed at a given position
         * @param position The current position in inches
         * @return The target velocity in inches per second
         */
        virtual double getSpeed(double position) = 0;
    };
}