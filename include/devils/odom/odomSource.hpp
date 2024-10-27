#pragma once
#include "../geometry/pose.hpp"

namespace devils
{
    /**
     * Represents a source of odometry data.
     */
    struct OdomSource
    {
        /**
         * Gets the current pose of the robot
         * @return The current pose of the robot
         */
        virtual Pose &getPose() = 0;

        /**
         * Sets the current pose of the robot
         * @param pose The pose to set the robot to
         */
        virtual void setPose(Pose &pose) = 0;
    };
}