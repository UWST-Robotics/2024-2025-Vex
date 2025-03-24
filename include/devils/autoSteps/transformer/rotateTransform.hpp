#pragma once

#include "poseTransform.h"

namespace devils
{
    /// @brief Transforms the robot's pose by rotating it 180 degrees
    class RotateTransform : public PoseTransform
    {
        Pose transform(Pose pose) override
        {
            return Pose(
                -pose.x,
                -pose.y,
                M_PI + pose.rotation);
        }
    };
}