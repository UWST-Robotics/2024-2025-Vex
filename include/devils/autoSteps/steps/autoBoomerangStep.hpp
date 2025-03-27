#pragma once
#include "pros/rtos.hpp"
#include "../autoStep.hpp"
#include "../../chassis/chassisBase.hpp"
#include "../../odom/odomSource.hpp"
#include "../../path/path.hpp"
#include "autoDriveToStep.hpp"

namespace devils
{
    /**
     * Represents a step to drive to a pose using boomerang control.
     * Results in a curved path toward the target pose.
     * See: https://area-53-robotics.github.io/Intro-To-Robotics/software/advanced-concepts/boomerang/
     */
    class AutoBoomerangStep : public AutoDriveToStep
    {
    public:
        /**
         * Creates a new boomerang step.
         * @param chassis The chassis to control.
         * @param odomSource The odometry source to use.
         * @param pose The target pose to drive to.
         */
        AutoBoomerangStep(
            ChassisBase &chassis,
            OdomSource &odomSource,
            Pose targetPose,
            Options options = Options::defaultOptions)
            : AutoDriveToStep(chassis, odomSource, targetPose, options),
              targetPose(targetPose),
              odomSource(odomSource)
        {
        }

        void onStart() override
        {
            // Start the drive step
            AutoDriveToStep::onStart();
        }

        void onUpdate() override
        {
            // Get the current pose
            Pose currentPose = odomSource.getPose();

            // Get the distance to the target
            double distance = currentPose.distanceTo(targetPose);

            // Calculate the dot product
            double currentDotTarget = cos(currentPose.rotation) * (targetPose.x - currentPose.x) +
                                      sin(currentPose.rotation) * (targetPose.y - currentPose.y);

            // Drive in reverse if the goal is behind us
            if (currentDotTarget < 0)
                distance = -distance;

            // Calculate the carrot pose
            Pose carrotPose = Pose(
                targetPose.x - distance * cos(targetPose.rotation) * LEAD_DISTANCE,
                targetPose.y - distance * sin(targetPose.rotation) * LEAD_DISTANCE,
                targetPose.rotation);

            // Set the target pose
            AutoDriveToStep::targetPose = carrotPose;

            // Update the drive step
            AutoDriveToStep::onUpdate();
        }

        void onStop() override
        {
            // Stop the drive step
            AutoDriveToStep::onStop();
        }

        bool checkFinished() override
        {
            // Check against the target pose, not the carrot pose
            AutoDriveToStep::targetPose = targetPose;

            return AutoDriveToStep::checkFinished();
        }

    protected:
        static constexpr double LEAD_DISTANCE = 0.7; // %

        Pose targetPose;
        OdomSource &odomSource;
    };
}