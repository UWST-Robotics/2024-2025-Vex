#pragma once

#include <vector>
#include "path.hpp"
#include "structs/splinePose.hpp"
#include "../geometry/lerp.hpp"

namespace devils
{
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

        double getLength() override
        {
            return poses.size();
        }

    private:
        std::vector<SplinePose> poses;
    };
}