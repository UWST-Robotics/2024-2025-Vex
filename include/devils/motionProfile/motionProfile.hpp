#pragma once

#include "../geometry/poseVelocity.hpp"

namespace devils
{
    /**
     *  Represents a 1-dimensional motion profile
     */
    struct MotionProfile
    {
        /**
         * Gets the target velocity given the current velocity and goal distance
         * @param currentPoseVelocity The current velocity as a `PoseVelocity`
         * @param goalDistance The goal distance in inches
         * @return The output speed in inches per second
         */
        virtual double getSpeed(double goalDistance) = 0;
    };
}