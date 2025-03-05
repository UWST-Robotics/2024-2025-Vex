#pragma once

#include "../../geometry/pose.hpp"

namespace devils
{
    /**
     * Used to transform pose depending on a set of transformations
     */
    struct AutoTransform
    {
        /**
         * Transforms the pose to another location
         * @param pose - Input Pose
         * @returns Output pose
         */
        virtual Pose transform(Pose pose) = 0;
    };
}