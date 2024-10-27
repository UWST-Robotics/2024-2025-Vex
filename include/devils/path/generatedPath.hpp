#pragma once
#include "pathFileReader.hpp"
#include "pathFile.hpp"
#include "../geometry/pose.hpp"
#include "../geometry/units.hpp"
#include <vector>

namespace devils
{
    /**
     * Represents a path generated from a series of control points.
     */
    struct GeneratedPath
    {
        /// @brief The amount of indices between each point in the path.
        double dt = 0;

        /// @brief A list of control points that the path was generated from.
        ControlPoints controlPoints = {};

        /// @brief The list of points interpolated between the control points. `1/dt` points per control point.
        PoseSequence pathPoints = {};

        /// @brief The indices of each control point in the path. `controlPointIndices[i]` is the index of the `i`th control point in the path.
        std::vector<int> controlPointIndices = {};

        /**
         * Gets the starting pose of the motion profile.
         * @return The starting pose of the motion profile as an Pose.
         */
        Pose *getStartingPose()
        {
            if (controlPoints.size() <= 0)
            {
                Logger::warn("No control points in path");
                return nullptr;
            }

            return &controlPoints[0];
        }

        /**
         * Gets whether the motion profile has been generated.
         * @return Whether the motion profile has been generated.
         */
        bool isGenerated()
        {
            return pathPoints.size() > 0;
        }
    };
}