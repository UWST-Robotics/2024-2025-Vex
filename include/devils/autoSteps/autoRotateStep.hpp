#pragma once
#include "pros/rtos.hpp"
#include "common/autoStep.hpp"
#include "devils/autoSteps/autoRotateToStep.hpp"
#include "devils/odom/odomSource.hpp"
#include "devils/chassis/chassisBase.hpp"
#include "devils/utils/math.hpp"

namespace devils
{
    /**
     * Represents a rotational step in an autonomous routine.
     */
    class AutoRotateStep : public AutoRotateToStep
    {
    public:
        /**
         * Creates a new rotational step.
         * @param chassis The chassis to control.
         * @param odomSource The odometry source to use.
         * @param distance The distance to rotate in radians.
         */
        AutoRotateStep(ChassisBase &chassis, OdomSource &odomSource, double distance)
            : AutoRotateToStep(chassis, odomSource, distance),
              distance(distance)
        {
        }

        void doStep() override
        {
            // Calculate Target Pose
            Pose startPose = odomSource.getPose();
            this->targetAngle = distance + startPose.rotation;

            // Do base step
            AutoRotateToStep::doStep();
        }

    protected:
        // Drive Step Variables
        double distance = 0;
    };
}