#pragma once

#include "pose.hpp"

namespace devils
{
    /**
     * Represents a velocity in 2D space.
     * Instead of inches or radians, values are in inches per second and radians per second.
     */
    typedef Pose PoseVelocity;
}