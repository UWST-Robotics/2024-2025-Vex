#pragma once
#include "pros/rtos.hpp"
#include "autoRotateToStep.hpp"
#include "../common/autoStep.hpp"
#include "../../odom/odomSource.hpp"
#include "../../chassis/chassisBase.hpp"
#include "../../utils/math.hpp"

namespace devils
{
    // Forward Declaration
    class AbsoluteStepConverter;

    /**
     * Rotates the robot toward a specific pose along its center of rotation.
     */
    class AutoRotateTowardStep : public AutoRotateToStep
    {
        // Allow the absolute step converter to access private members
        friend class AbsoluteStepConverter;

    public:
        /**
         * Rotates the robot toward a specific pose along its center of rotation.
         * @param chassis The chassis to control.
         * @param odomSource The odometry source to use.
         * @param targetPose The target pose to rotate toward.
         * @param options The options for the rotational step.
         */
        AutoRotateTowardStep(
            ChassisBase &chassis,
            OdomSource &odomSource,
            Pose targetPose,
            Options options = Options::defaultOptions)
            : AutoRotateToStep(chassis, odomSource, 0, options),
              targetPose(targetPose)
        {
        }

        void onStart() override
        {
            // Calculate Target Pose
            Pose startPose = odomSource.getPose();
            this->targetAngle = std::atan2(
                targetPose.y - startPose.y,
                targetPose.x - startPose.x);

            // Do base step
            AutoRotateToStep::onStart();
        }

    protected:
        Pose targetPose;
    };
}