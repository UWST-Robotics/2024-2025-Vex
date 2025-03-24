#pragma once
#include "pros/rtos.hpp"
#include "../autoStep.hpp"
#include "../../odom/odomSource.hpp"

namespace devils
{
    /**
     * Jumps the odometry state to a given pose. Usually ran at the start of an auto routine.
     */
    class AutoJumpToStep : public AutoStep
    {
    public:
        /**
         * Jumps the odometry state to a given pose. Usually ran at the start of an auto routine.
         * @param chassis The chassis to control.
         * @param x The x position to jump to in inches.
         * @param y The y position to jump to in inches.
         * @param heading The heading to jump to in radians.
         */
        AutoJumpToStep(OdomSource &odom, double x, double y, double heading)
            : odom(odom),
              targetPose(x, y, heading)
        {
        }

        /**
         * Jumps the odometry state to a given pose. Usually ran at the start of an auto routine.
         * @param chassis The chassis to control.
         * @param pose The pose to jump to.
         */
        AutoJumpToStep(OdomSource &odom, Pose pose)
            : odom(odom),
              targetPose(pose)
        {
        }

        void onStart() override
        {
            odom.setPose(targetPose);
        }

        bool checkFinished() override
        {
            return true;
        }

    protected:
        // Params
        OdomSource &odom;
        Pose targetPose;
    };
}