#pragma once

#include "poseTransform.h"

namespace devils
{
    /// @brief Transforms the robot's pose by mirroring it over the x-axis
    class MirrorTransformX : public PoseTransform
    {
        Pose transform(Pose pose) override
        {
            return Pose(
                -pose.x,
                pose.y,
                M_PI - pose.rotation);
        }
    };

    /// @brief Transforms the robot's pose by mirroring it over the y-axis
    class MirrorTransformY : public PoseTransform
    {
        Pose transform(Pose pose) override
        {
            return Pose(
                pose.x,
                -pose.y,
                -pose.rotation);
        }
    };
}