#pragma once

#include <vector>
#include "path.hpp"
#include "../geometry/pose.hpp"
#include "../geometry/lerp.hpp"

namespace devils
{
    class LinearPath : public Path
    {
    public:
        /**
         * Creates a new instance of a linear path using a start and end pose.
         * @param startPose The starting pose of the path
         * @param endPose The ending pose of the path
         */
        LinearPath(Pose startPose, Pose endPose)
            : startPose(startPose),
              endPose(endPose)
        {
        }

        Pose getPoseAt(double index) override
        {
            // Check OOB
            if (index <= 0)
                return startPose;
            if (index >= 1)
                return endPose;

            // Interpolate between the two poses
            return Lerp::linearPoints(startPose, endPose, index);
        }

        double getLength() override
        {
            return 2;
        }

    private:
        Pose startPose;
        Pose endPose;
    };
}