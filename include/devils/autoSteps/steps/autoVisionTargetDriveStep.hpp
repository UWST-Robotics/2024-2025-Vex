#pragma once
#include <string>
#include "pros/rtos.hpp"
#include "../common/autoStep.hpp"
#include "../../hardware/visionSensor.hpp"
#include "../../utils/math.hpp"
#include "../../odom/odomSource.hpp"
#include "../../chassis/chassisBase.hpp"
#include "../../vision/structs/visionTargetProvider.h"
#include "autoDriveToStep.hpp"

namespace devils
{
    /**
     * Drives the robot to a vision target w/ odometry feedback.
     */
    class AutoVisionTargetDriveStep : public AutoDriveToStep
    {
    public:
        /**
         * Drives the robot to a vision target w/ odometry feedback.
         * @param chassis The chassis to control.
         * @param odomSource The odometry source to use.
         * @param defaultPose The default pose to drive to.
         * @param visionTarget The vision target to drive towards.
         * @param options The options for the drive step.
         */
        AutoVisionTargetDriveStep(
            ChassisBase &chassis,
            OdomSource &odomSource,
            Pose defaultPose,
            IVisionTargetProvider &visionTargetProvider,
            Options options = Options::defaultOptions)
            : AutoDriveToStep(chassis, odomSource, defaultPose, options),
              visionTargetProvider(visionTargetProvider),
              defaultPose(defaultPose)
        {
        }

        void onUpdate() override
        {
            // Check if target is visible
            if (visionTargetProvider.hasTargets())
            {
                // Get target pose
                Pose targetPose = visionTargetProvider.getClosestTarget();

                // Drive to target
                this->targetPose = targetPose;
            }
            else
            {
                // Drive to default pose
                this->targetPose = defaultPose;
            }

            // Do base step
            AutoDriveToStep::onUpdate();
        }

    protected:
        // Drive Step Variables
        Pose defaultPose;
        IVisionTargetProvider &visionTargetProvider;
    };
}