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
    class AutoRotateToStep : public IAutoStep
    {
    public:
        struct Options
        {
            /// @brief The distance to start accelerating in rads
            double accelDist = M_PI * 0.1;

            /// @brief The distance to start decelerating in rads
            double decelDist = M_PI * 0.7;

            /// @brief The maximum speed in %
            double maxSpeed = 0.3;

            /// @brief The minimum speed in %
            double minSpeed = 0.15;

            /// @brief The distance to the goal in radians
            double goalDist = 0.01;

            /// @brief Whether to use the minimum distance between the start and target
            bool useMinimumDistance = true;

            /// @brief The default options for the rotational step.
            static Options getDefault()
            {
                return Options();
            }
        };

        /**
         * Creates a new rotational step.
         * @param chassis The chassis to control.
         * @param odomSource The odometry source to use.
         * @param targetAngle The angle to rotate to in radians.
         * @param options The options for the rotational step.
         */
        AutoRotateToStep(
            ChassisBase &chassis,
            OdomSource &odomSource,
            double targetAngle,
            Options options = Options::getDefault())
            : chassis(chassis),
              odomSource(odomSource),
              targetAngle(targetAngle),
              options(options)
        {
        }

        void doStep() override
        {
            // Calculate Target Pose
            Pose startPose = odomSource.getPose();
            double startAngle = startPose.rotation;

            // Control Loop
            while (true)
            {
                // Calculate distance to start and target
                Pose currentPose = odomSource.getPose();
                double currentAngle = currentPose.rotation;
                double distanceToStart = angleDiff(startAngle, currentAngle);
                double distanceToTarget = angleDiff(targetAngle, currentAngle);

                // Calculate Speed
                double speed = Math::trapezoidProfile(
                    distanceToStart,
                    distanceToTarget,
                    options.accelDist,
                    options.decelDist,
                    options.minSpeed,
                    options.maxSpeed);

                // Check if we are at the target
                if (fabs(distanceToTarget) < options.goalDist)
                    break;

                // Move Chassis
                chassis.move(0, speed);

                // Delay
                pros::delay(10);
            }

            // Stop Chassis
            chassis.stop();

            // Delay
            pros::delay(POST_DRIVE_DELAY);
        }

        Options &getOptions()
        {
            return options;
        }

    protected:
        // Options
        Options options;

        // Robot Base
        ChassisBase &chassis;
        OdomSource &odomSource;

        // Drive Step Variables
        double targetAngle = 0;

    private:
        static constexpr double POST_DRIVE_DELAY = 500; // ms

        /**
         * Gets the angle difference between two angles.
         * Uses the minimum distance if the option is enabled.
         * Otherwise, returns the difference.
         * @param a The first angle.
         * @param b The second angle.
         * @return The difference between the two angles.
         */
        double angleDiff(double a, double b)
        {
            if (options.useMinimumDistance)
                return Math::angleDiff(a, b);
            return a - b;
        }
    };
}
