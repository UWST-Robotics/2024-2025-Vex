#pragma once
#include "autoRotateStep.hpp"

namespace devils
{
    /**
     * Represents a rotational step in an autonomous routine.
     */
    class AutoRotateToStep : public AutoRotateStep
    {
    public:
        /**
         * Creates a new rotational step.
         * @param chassis The chassis to control.
         * @param odomSource The odometry source to use.
         * @param angle The absolute angle to rotate to in radians.
         */
        AutoRotateToStep(ChassisBase &chassis, OdomSource &odomSource, double angle)
            : AutoRotateStep(chassis, odomSource, angle),
              angle(angle)
        {
        }

        void doStep() override
        {
            // Calculate relative angle
            Pose currentPose = odomSource.getPose();
            double currentAngle = currentPose.rotation;
            double angleDiff = Math::angleDiff(angle, currentAngle);
            distance = angleDiff;

            // Do base step
            AutoRotateStep::doStep();
        }

    private:
        // Drive Step Variables
        double angle = 0;
    };
}