#pragma once
#include <algorithm>

namespace devils
{
    /**
     * Represents a base chassis. A chassis is a robot's drivetrain.
     */
    struct BaseChassis
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

        /**
         * Sets the speed of the robot.
         * @param forwardSpeed The forward speed of the robot from -1 to 1.
         * @param turnSpeed The turn speed of the robot from -1 to 1.
         * @param strafeSpeed The strafe speed of the robot from -1 to 1.
         */
        virtual void setSpeed(double forwardSpeed, double turnSpeed, double strafeSpeed = 0)
        {
            this->forwardSpeed = std::clamp(forwardSpeed, 0.0, 1.0);
            this->turnSpeed = std::clamp(turnSpeed, 0.0, 1.0);
            this->strafeSpeed = std::clamp(strafeSpeed, 0.0, 1.0);
        }

    protected:
        double forwardSpeed = 1.0;
        double turnSpeed = 1.0;
        double strafeSpeed = 1.0;
    };
}