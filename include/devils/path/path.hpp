#pragma once

#include <vector>
#include "../geometry/pose.hpp"

namespace devils
{
    /**
     * Represents a continuous list of connected points that can be interpolated.
     */
    struct Path
    {
        /**
         * Gets an interpolated pose at a given index
         * @param index The index of the pose
         * @return An interpolated pose at the given index
         */
        virtual Pose getPoseAt(double index) = 0;

        /**
         * Gets the length of the path
         * @return The length of the path in indices
         */
        virtual double getLength() = 0;
    };
}