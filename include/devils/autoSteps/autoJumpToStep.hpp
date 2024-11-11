#pragma once
#include "pros/rtos.hpp"
#include "common/autoStep.hpp"
#include "devils/odom/odomSource.hpp"

namespace devils
{
    /**
     * Represents a pause step in an autonomous routine.
     */
    class AutoJumpToStep : public IAutoStep
    {
    public:
        /**
         * Creates a new pause step.
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

        void doStep() override
        {
            odom.setPose(targetPose);
        }

    protected:
        OdomSource &odom;
        Pose targetPose;
    };
}