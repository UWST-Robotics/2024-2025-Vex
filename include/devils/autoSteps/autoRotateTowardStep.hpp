#pragma once
#include "pros/rtos.hpp"
#include "common/autoStep.hpp"
#include "devils/autoSteps/autoRotateToStep.hpp"
#include "devils/odom/odomSource.hpp"
#include "devils/chassis/chassisBase.hpp"
#include "devils/utils/math.hpp"

namespace devils
{
    // Forward Declaration
    class AbsoluteStepConverter;

    /**
     * Represents a rotational step in an autonomous routine.
     */
    class AutoRotateTowardStep : public AutoRotateToStep
    {
        // Allow the absolute step converter to access private members
        friend class AbsoluteStepConverter;

    public:
        /**
         * Creates a new rotational step.
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

        void doStep() override
        {
            // TODO: Test & Fix me
            // Calculate Target Pose
            Pose startPose = odomSource.getPose();
            this->targetAngle = std::atan2(
                targetPose.y - startPose.y,
                targetPose.x - startPose.x);

            // Do base step
            AutoRotateToStep::doStep();
        }

    protected:
        Pose targetPose;
    };
}