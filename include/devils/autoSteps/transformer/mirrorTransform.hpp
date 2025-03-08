#pragma once

#include "autoTransform.hpp"

namespace devils
{
    struct MirrorTransform : public AutoTransform
    {
        Pose transform(Pose pose) override
        {
            return Pose(
                -pose.x,
                pose.y,
                M_PI - pose.rotation);
        }
    };
}