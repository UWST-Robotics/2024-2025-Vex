#pragma once
#include "generatedPath.hpp"
#include "../geometry/lerp.hpp"
#include <vector>

namespace devils
{
    struct PathGenerator
    {
        /**
         * Generates a path from a set of control points using linear interpolation.
         * @return The generated path.
         */
        static GeneratedPath generateLinear(PathFile pathFile)
        {
            // Get path file
            auto controlPoints = pathFile.points;

            // Initialize path points
            std::vector<int> controlPointIndices;
            PoseSequence pathPoints;
            pathPoints.reserve(controlPoints.size() * (1 / DT));

            // Iterate through each set of control points
            for (int i = 0; i < controlPoints.size() - 1; i++)
            {
                // Add Control Point Index
                controlPointIndices.push_back(pathPoints.size());

                // Get the two points to lerp between
                auto p1 = controlPoints[i];
                auto p2 = controlPoints[i + 1];

                // Lerp between points
                for (double t = 0; t < 1; t += DT)
                    pathPoints.push_back(Lerp::linearPoints(p1, p2, t));
            }
            controlPointIndices.push_back(pathPoints.size() - 1);

            // Return the generated path
            return GeneratedPath{
                DT,
                controlPoints,
                pathPoints,
                controlPointIndices};
        }

        /**
         * Generates a path from a set of control points using cubic interpolation.
         * @return The generated path.
         */
        static GeneratedPath generateSpline(PathFile pathFile)
        {
            // Get path file
            auto controlPoints = pathFile.points;

            // Initialize path points
            std::vector<int> controlPointIndices;
            PoseSequence pathPoints;
            pathPoints.reserve(controlPoints.size() * (1 / DT));

            // Iterate through each set of control points
            bool isReversed = false;
            for (int i = 0; i < controlPoints.size() - 1; i++)
            {
                // Add Control Point Index
                controlPointIndices.push_back(pathPoints.size());

                // Get the two points to lerp between
                ControlPoint &p1 = controlPoints[i];
                ControlPoint &p2 = controlPoints[i + 1];

                // Get Anchor Points
                Pose a1 = Pose(
                    p1.x + p1.exitDelta * std::cos(p1.rotation) * (isReversed ? -1 : 1),
                    p1.y + p1.exitDelta * std::sin(p1.rotation) * (isReversed ? -1 : 1),
                    p1.rotation);
                Pose a2 = Pose(
                    p2.x - p2.enterDelta * std::cos(p2.rotation),
                    p2.y - p2.enterDelta * std::sin(p2.rotation),
                    p2.rotation);

                // Lerp between points
                for (double t = 0; t < 1; t += DT)
                    pathPoints.push_back(Lerp::cubicPoints(p1, a1, a2, p2, t));

                // Reverse Anchor Points
                if (p2.isReversed)
                    isReversed = !isReversed;
            }
            controlPointIndices.push_back(pathPoints.size() - 1);

            // Return the generated path
            return GeneratedPath{
                DT,
                controlPoints,
                pathPoints,
                controlPointIndices};
        }

    private:
        PathGenerator() = delete;

        static constexpr double DT = 0.025; // indices between each point in the path
    };
}