#pragma once
#include <string>
#include "pros/rtos.hpp"
#include "../autoStep.hpp"
#include "../../utils/math.hpp"
#include "../../odom/odomSource.hpp"
#include "../../chassis/chassisBase.hpp"
#include "autoBoomerangStep.hpp"

namespace devils
{
    /**
     * Drives the robot a to a specific pose using a 1D motion profile for acceleration/deceleration.
     */
    class AutoDriveMotionProfileStep : public AutoBoomerangStep
    {
    public:
        /**
         * Drives the robot a specific pose using a 1D motion profile for acceleration/deceleration.
         * @param chassis The chassis to control.
         * @param odomSource The odometry source to use.
         * @param targetPose The target pose to drive to.
         * @param motionProfile The motion profile to use for acceleration/deceleration.
         * @param options The options for the drive step.
         */
        AutoDriveMotionProfileStep(
            ChassisBase &chassis,
            OdomSource &odomSource,
            Pose targetPose,
            std::shared_ptr<MotionProfile> motionProfile,
            Options options = Options::defaultOptions)
            : AutoBoomerangStep(chassis, odomSource, targetPose, options),
              motionProfile(motionProfile)
        {
        }

    protected:
        double getSpeed(double distanceToGoal) override
        {
            double targetVelocity = motionProfile->getSpeed(distanceToGoal);
            double currentVelocity = odomSource.getVelocity().magnitude();
            double outputSpeed = translationPID.update(targetVelocity - currentVelocity);
            return std::clamp(outputSpeed, options.minSpeed, options.maxSpeed);
        }

        std::shared_ptr<MotionProfile> motionProfile;
    };
}