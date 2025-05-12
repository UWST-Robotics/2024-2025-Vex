#pragma once
#include <algorithm>

namespace devils
{
    /**
     * Represents a chassis base. A chassis is a robot's drivetrain.
     */
    struct ChassisBase
    {
        /**
         * Moves the robot in a direction using voltage.
         * @param forward The forward speed of the robot from -1 to 1.
         * @param turn The turn speed of the robot from -1 to 1.
         * @param strafe The strafe speed of the robot from -1 to 1.
         */
        virtual void move(double forward, double turn, double strafe = 0) = 0;

        /**
     * Stops the robot.
         */
        virtual void stop()
        {
            move(0, 0, 0);
        };
    };
}