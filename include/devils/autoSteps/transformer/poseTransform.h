#pragma once

#include "../../geometry/pose.hpp"

namespace devils
{
    /**
     * Base class for transforming poses.
     * Used to reuse autonomous steps across different alliance orientations.
     */
    struct PoseTransform
    {
        /**
         * Transforms the pose to another location
         * @param pose - Input Pose
         * @returns Output pose
         */
        virtual Pose transform(Pose pose) = 0;
    };
}