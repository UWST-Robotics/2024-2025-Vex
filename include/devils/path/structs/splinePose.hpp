#pragma once

#include "../../geometry/pose.hpp"

namespace devils
{
    /**
     * Represents a pose with additional spline data.
     * Used in `SplinePath` to define control points.
     */
    struct SplinePose : public Pose
    {
        SplinePose() : Pose() {}

        SplinePose(
            double x,
            double y,
            double rotation,
            double entryDelta,
            double exitDelta)
            : Pose(x, y, rotation),
              entryDelta(entryDelta),
              exitDelta(exitDelta) {}

        /// @brief Distance of anchor point in inches from the start of the spline
        double entryDelta = 0;

        /// @brief Distance of anchor point in inches from the end of the spline
        double exitDelta = 0;

        /**
         * Gets the pose of the entry anchor point of the spline
         * @return The entry anchor pose
         */
        Pose getEntryAnchor()
        {
            return Pose(
                x + std::cos(rotation) * -entryDelta,
                y + std::sin(rotation) * -entryDelta,
                rotation);
        }

        /**
         * Gets the pose of the exit anchor point of the spline
         * @return The exit anchor pose
         */
        Pose getExitAnchor()
        {
            return Pose(
                x + std::cos(rotation) * exitDelta,
                y + std::sin(rotation) * exitDelta,
                rotation);
        }
    };
}