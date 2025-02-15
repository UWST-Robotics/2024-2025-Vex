#pragma once

#include <vector>
#include "path.hpp"
#include "structs/splinePose.hpp"
#include "../geometry/lerp.hpp"

namespace devils
{
    /**
     * Represents a path that is interpolated using cubic splines.
     */
    class SplinePath : public Path
    {
    public:
        /**
         * Creates a new instance of a spline path using a list of SplinePoses.
         * @param poses The list of poses to interpolate between
         * @return The spline path
         */
        SplinePath(std::vector<SplinePose> poses)
        {
            this->poses = poses;
        }

        /**
         * Makes a simple 2 point arc path.
         * @param from The starting pose
         * @param to The ending pose
         * @param delta Distance of the entry and exit anchor points in inches
         * @return The spline path
         */
        static SplinePath *makeArc(Pose from, Pose to, double delta = 18.0)
        {
            std::vector<SplinePose> poses;
            poses.push_back(SplinePose(from.x, from.y, from.rotation, delta, delta));
            poses.push_back(SplinePose(to.x, to.y, to.rotation, delta, delta));
            return new SplinePath(poses);
        }

        /**
         * Gets a pose at a specific index along the path.
         * @param index The index to get the pose at. Interpolate between indices.
         * @return The pose at the index
         */
        Pose getPoseAt(double index) override
        {
            // Check OOB
            if (index <= 0)
                return poses.front();
            if (index >= poses.size() - 1)
                return poses.back();

            // Get the two poses to interpolate between
            int prevIndex = (int)index;
            int nextIndex = prevIndex + 1;
            SplinePose prevPose = poses[prevIndex];
            SplinePose nextPose = poses[nextIndex];
            Pose prevAnchor = prevPose.getExitAnchor();
            Pose nextAnchor = nextPose.getEntryAnchor();

            // Calculate dt between the two poses
            // This is the percentage of the way between the two poses
            double dt = index - prevIndex;

            // Interpolate between the two poses
            // using cubic interpolation
            return Lerp::cubicPoints(
                prevPose,
                prevAnchor,
                nextAnchor,
                nextPose,
                dt);
        }

        /**
         * Gets the length of the path
         * @return The length of the path in control points
         */
        double getLength() override
        {
            return poses.size();
        }

    private:
        std::vector<SplinePose> poses;
    };
}