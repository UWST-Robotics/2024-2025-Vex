#pragma once
#include <string>
#include "pros/rtos.hpp"
#include "common/autoStep.hpp"
#include "devils/odom/odomSource.hpp"
#include "devils/chassis/chassisBase.hpp"

namespace devils
{
    /**
     * Represents a drive step in an autonomous routine.
     */
    class AutoDriveStep : public IAutoStep
    {
    public:
        struct Options
        {
            /// @brief The distance to start accelerating in inches
            double accelDist = 12.0;

            /// @brief The distance to start decelerating in inches
            double decelDist = 12.0;

            /// @brief The maximum speed in %
            double maxSpeed = 0.8;

            /// @brief The minimum speed in %
            double minSpeed = 0.1;

            /// @brief The gain for rotation in %/rad
            double rotationGain = 2.0;

            /// @brief The distance to the goal in inches
            double goalDist = 3.0;
        };

        /**
         * Creates a new drive step.
         * @param chassis The chassis to control.
         * @param odomSource The odometry source to use.
         * @param distance The distance to drive in inches.
         * @param options The options for the drive step.
         */
        AutoDriveStep(ChassisBase &chassis, OdomSource &odomSource, double distance, Options options = Options())
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
            Pose deltaPose = Pose(
                distance * cos(startPose.rotation),
                distance * sin(startPose.rotation),
                0);
            Pose targetPose = startPose + deltaPose;

            // Control Loop
            while (true)
            {
                // Calculate distance to start and target
                Pose currentPose = odomSource.getPose();
                double distanceToStart = currentPose.distanceTo(startPose);
                double distanceToTarget = currentPose.distanceTo(targetPose);

                // Check if we are at the target
                if (distanceToTarget < options.goalDist)
                    break;

                // Calculate Speed
                // Uses a trapezoidal motion profile
                double accelPercent = std::clamp(distanceToStart / options.accelDist, 0.0, 1.0);  // Percent of distance to start
                double decelPercent = std::clamp(distanceToTarget / options.decelDist, 0.0, 1.0); // Percent of distance to target
                double speedPercent = std::min(accelPercent, decelPercent);                       // Use the smaller of the two
                double speed = std::lerp(options.minSpeed, options.maxSpeed, speedPercent);       // Lerp between min and max speed

                // Calculate Angle
                double targetAngleRads = std::atan2(
                    targetPose.y - currentPose.y,
                    targetPose.x - currentPose.x);
                double angleDiff = targetAngleRads - currentPose.rotation;
                double turnSpeed = options.rotationGain * angleDiff;

                // Move Chassis
                chassis.move(speed, turnSpeed);

                // Delay
                pros::delay(10);
            }

            // Stop
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