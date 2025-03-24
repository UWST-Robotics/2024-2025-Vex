#pragma once
#include "pros/rtos.hpp"
#include "autoRotateToStep.hpp"
#include "../autoStep.hpp"
#include "../../odom/odomSource.hpp"
#include "../../chassis/chassisBase.hpp"
#include "../../utils/math.hpp"

namespace devils
{
    /**
     * Rotates the robot a specific distance along its center of rotation.
     */
    class AutoRotateStep : public AutoRotateToStep
    {
    public:
        /**
         * Rotates the robot a specific distance along its center of rotation.
         * @param chassis The chassis to control.
         * @param odomSource The odometry source to use.
         * @param distance The distance to rotate in radians.
         * @param options The options for the rotational step.
         */
        AutoRotateStep(
            ChassisBase &chassis,
            OdomSource &odomSource,
            double distance,
            Options options = Options::defaultOptions)
            : AutoRotateToStep(chassis, odomSource, distance, options),
              distance(distance)
        {
        }

        void onStart() override
        {
            // Calculate Target Pose
            Pose startPose = odomSource.getPose();
            this->targetAngle = distance + startPose.rotation;

            // Do base step
            AutoRotateToStep::onStart();
        }

    protected:
        // Drive Step Variables
        double distance = 0;
    };
}