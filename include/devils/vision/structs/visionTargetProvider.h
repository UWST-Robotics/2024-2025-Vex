#pragma once
#include "../geometry/pose.hpp"

namespace devils
{
    /**
     * Base class for a vision pipeline.
     */
    struct IVisionTargetProvider
    {
        virtual bool hasTargets() = 0;
        virtual Pose getClosestTarget() = 0;
    };
}