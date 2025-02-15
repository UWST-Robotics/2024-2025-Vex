#pragma once

namespace devils
{
    /**
     * Represents some kind of gyroscopic sensor that can measure orientation.
     */
    struct IGyro
    {
        /**
         * Gets the current heading of the sensor in radians.
         * @return The current heading of the sensor in radians.
         */
        virtual double getHeading() = 0;
    };
}