#pragma once

#include <vector>
#include "path.hpp"
#include "../geometry/pose.hpp"
#include "../geometry/lerp.hpp"

namespace devils
{
    /**
     * Linearly interpolated path
     */
    class LinearPath : public Path
    {
    public:
        /**
         * Creates a new instance of a linear path using a list of poses.
         * @param poses The list of poses to interpolate between
         */
        LinearPath(std::vector<Pose> poses)
            : poses(poses)
        {
        }

        Pose getPoseAt(double index) override
        {
            // Check OOB
            if (index <= 0)
                return poses.front();
            if (index >= poses.size() - 1)
                return poses.back();

            // Get the two poses to interpolate between
            Pose prevPose = poses[(int)index];
            Pose nextPose = poses[(int)index + 1];

            // Interpolate between the two poses
            double t = index - (int)index;
            return Pose::lerp(prevPose, nextPose, t);
        }

        double getLength() override
        {
            return poses.size();
        }

    private:
        std::vector<Pose> poses;
    };
}