#pragma once
#include "pros/rtos.hpp"
#include "common/autoStep.hpp"
#include "devils/odom/odomSource.hpp"
#include "devils/chassis/chassisBase.hpp"
#include "devils/utils/math.hpp"

namespace devils
{
    /**
     * Represents a rotational step in an autonomous routine.
     */
    class AutoRotateStep : public IAutoStep
    {
    public:
        struct Options
        {
            /// @brief The distance to start accelerating in rads
            double accelDist = 1.0;

            /// @brief The distance to start decelerating in rads
            double decelDist = 1.0;

            /// @brief The maximum speed in %
            double maxSpeed = 0.8;

            /// @brief The minimum speed in %
            double minSpeed = 0.1;

            /// @brief The distance to the goal in radians
            double goalDist = 0.1;
        };

        /**
         * Creates a new rotational step.
         * @param chassis The chassis to control.
         * @param odomSource The odometry source to use.
         * @param distance The distance to rotate in radians.
         * @param options The options for the rotational step.
         */
        AutoRotateStep(ChassisBase &chassis, OdomSource &odomSource, double distance, Options options = Options())
            : chassis(chassis),
              odomSource(odomSource),
              distance(distance),
              options(options)
        {
        }

        void doStep() override
        {
            // Calculate Target Pose
            Pose startPose = odomSource.getPose();
            double startAngle = startPose.rotation;
            double targetAngle = startPose.rotation + distance;

            // Control Loop
            while (true)
            {
                // Calculate distance to start and target
                Pose currentPose = odomSource.getPose();
                double currentAngle = currentPose.rotation;
                double distanceToStart = Math::angleDiff(currentAngle, startAngle);
                double distanceToTarget = Math::angleDiff(currentAngle, targetAngle);

                // Check if we are at the target
                if (distanceToTarget < options.goalDist)
                    break;

                // Calculate Speed
                double speed = Math::trapezoidProfile(
                    distanceToStart,
                    distanceToTarget,
                    options.accelDist,
                    options.decelDist,
                    options.minSpeed,
                    options.maxSpeed);

                // Move Chassis
                chassis.move(0, speed);

                // Delay
                pros::delay(10);
            }

            // Stop Chassis
            chassis.stop();
        }

        Options &getOptions()
        {
            return options;
        }

    private:
        // Options
        Options options;

        // Robot Base
        ChassisBase &chassis;
        OdomSource &odomSource;

        // Drive Step Variables
        double distance = 0;
    };
}