#pragma once
#include <string>
#include "pros/rtos.hpp"
#include "common/autoStep.hpp"
#include "devils/utils/math.hpp"
#include "devils/odom/odomSource.hpp"
#include "devils/chassis/chassisBase.hpp"
#include "devils/autoSteps/autoDriveToStep.hpp"

namespace devils
{
    // Forward Declaration
    class AbsoluteStepConverter;

    /**
     * Represents a drive step in an autonomous routine.
     */
    class AutoDriveStep : public AutoDriveToStep
    {
        // Allow the absolute step converter to access private members
        friend class AbsoluteStepConverter;

    public:
        /**
         * Creates a new drive step.
         * @param chassis The chassis to control.
         * @param odomSource The odometry source to use.
         * @param distance The distance to drive in inches.
         * @param options The options for the drive step.
         */
        AutoDriveStep(
            ChassisBase &chassis,
            OdomSource &odomSource,
            double distance,
            Options options = Options::defaultOptions)
            : AutoDriveToStep(chassis, odomSource, Pose(0, 0, 0), options),
              distance(distance)
        {
        }

        void doStep() override
        {
            // Calculate Target Pose
            Pose startPose = odomSource.getPose();
            this->targetPose = Pose(
                startPose.x + distance * std::cos(startPose.rotation),
                startPose.y + distance * std::sin(startPose.rotation),
                startPose.rotation);

            // Do base step
            AutoDriveToStep::doStep();
        }

    protected:
        // Drive Step Variables
        double distance = 0;
    };
}