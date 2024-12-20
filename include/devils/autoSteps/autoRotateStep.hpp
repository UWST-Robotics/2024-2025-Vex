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
    class AutoRotateStep : public AutoRotateToStep
    {
        // Allow the absolute step converter to access private members
        friend class AbsoluteStepConverter;

    public:
        /**
         * Creates a new rotational step.
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

        void doStep() override
        {
            // Calculate Target Pose
            Pose startPose = odomSource.getPose();
            this->targetAngle = distance + startPose.rotation;

            // Do base step
            AutoRotateToStep::doStep();
        }

        /**
         * Gets the distance of the step in radians.
         * @return The distance of the step in radians.
         */
        double getDistance()
        {
            return distance;
        }

    protected:
        // Drive Step Variables
        double distance = 0;
    };
}