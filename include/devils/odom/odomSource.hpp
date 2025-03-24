#pragma once

#include "../geometry/pose.hpp"
#include "../geometry/poseVelocity.hpp"

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

        /**
         * Gets the current velocity of the robot.
         * @return The current velocity of the robot as a `PoseVelocity`.
         */
        virtual PoseVelocity &getVelocity() = 0;
    };
}