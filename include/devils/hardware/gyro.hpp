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

        /**
         * Sets the current heading of the sensor in radians.
         * @param heading The heading to set the sensor to in radians.
         */
        virtual void setHeading(double heading) = 0;
    };
}