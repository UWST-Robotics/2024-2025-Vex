#pragma once
#include <string>
#include "pros/rtos.hpp"
#include "common/autoStep.hpp"
#include "devils/nt/networkTables.hpp"
#include "devils/utils/math.hpp"
#include "devils/odom/odomSource.hpp"
#include "devils/chassis/chassisBase.hpp"

namespace devils
{
    /**
     * Represents a drive step in an autonomous routine.
     */
    class AutoDriveToStep : public IAutoStep
    {
    public:
        struct Options
        {
            /// @brief The distance to start accelerating in inches
            double accelDist = 3.0;

            /// @brief The distance to start decelerating in inches
            double decelDist = 16.0;

            /// @brief The maximum speed in %
            double maxSpeed = 0.5;

            /// @brief The minimum speed in %
            double minSpeed = 0.15;

            /// @brief The gain for rotation in %/rad
            double rotationGain = 2.0;

            /// @brief The distance to the goal in inches
            double goalDist = 0.5;
        };

        /**
         * Creates a new drive step.
         * @param chassis The chassis to control.
         * @param odomSource The odometry source to use.
         * @param targetPose The target pose to drive to.
         * @param options The options for the drive step.
         */
        AutoDriveToStep(
            ChassisBase &chassis,
            OdomSource &odomSource,
            Pose targetPose,
            Options *options = nullptr)
            : chassis(chassis),
              odomSource(odomSource),
              targetPose(targetPose),
              options(options == nullptr ? Options() : *options)
        {
        }

        void doStep() override
        {
            // Calculate Target Pose
            Pose startPose = odomSource.getPose();

            // Control Loop
            while (true)
            {
                // Calculate distance to start and target
                Pose currentPose = odomSource.getPose();
                double distanceToStart = currentPose.distanceTo(startPose);
                double distanceToTarget = currentPose.distanceTo(targetPose);

                // Calculate Dot Product
                double dot = cos(currentPose.rotation) *
                                 (targetPose.x - currentPose.x) +
                             sin(currentPose.rotation) *
                                 (targetPose.y - currentPose.y);

                // Reverse if needed
                distanceToTarget *= std::copysign(1.0, dot);

                // Calculate Speed
                double speed = Math::trapezoidProfile(
                    distanceToStart,
                    distanceToTarget,
                    options.accelDist,
                    options.decelDist,
                    options.minSpeed,
                    options.maxSpeed);

                // Calculate Angle
                double targetAngleRads = std::atan2(
                    targetPose.y - currentPose.y,
                    targetPose.x - currentPose.x);

                // Apply direction
                if (dot < 0)
                    targetAngleRads += M_PI;

                // Calculate Angle Difference
                double angleDiff = Math::angleDiff(targetAngleRads, currentPose.rotation);

                // Calculate Turn Speed
                double turnSpeed = options.rotationGain * angleDiff;

                // Check if we are at the target
                if (fabs(distanceToTarget) < options.goalDist)
                    break;

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

    protected:
        // Options
        Options options = Options();

        // Robot Base
        ChassisBase &chassis;
        OdomSource &odomSource;

        // Drive Step Variables
        Pose targetPose = Pose();
    };
}