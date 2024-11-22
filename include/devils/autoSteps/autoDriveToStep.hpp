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
    // Forward Declaration
    class AbsoluteStepConverter;

    /**
     * Represents a drive step in an autonomous routine.
     */
    class AutoDriveToStep : public IAutoStep
    {
        // Allow the absolute step converter to access private members
        friend class AbsoluteStepConverter;

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
            double minSpeed = 0.18;

            /// @brief The gain for rotation in %/rad
            double rotationGain = 0.2;

            /// @brief The distance to the goal in inches
            double goalDist = 1.0;

            /// @brief The default options for the drive step.
            static Options getDefault()
            {
                return Options();
            }
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
            Options options = Options::getDefault())
            : chassis(chassis),
              odomSource(odomSource),
              targetPose(targetPose),
              options(options)
        {
        }

        void doStep() override
        {
            // Calculate Target Pose
            Pose startPose = odomSource.getPose();
            double totalDistance = startPose.distanceTo(targetPose);

            // Control Loop
            while (true)
            {
                // Calculate distance to start and target
                Pose currentPose = odomSource.getPose();
                double distanceToStart = currentPose.distanceTo(startPose);
                double distanceToTarget = Math::distanceOnLine(
                    startPose,
                    targetPose,
                    currentPose);

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

                // Calculate Distance Percent
                double distancePercent = std::fabs(distanceToTarget / totalDistance);
                distancePercent = std::clamp(distancePercent, 0.0, 1.0);

                // Calculate
                turnSpeed *= distancePercent;
                turnSpeed = std::clamp(turnSpeed, -options.maxSpeed, options.maxSpeed);

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

            pros::delay(POST_DRIVE_DELAY);
        }

    protected:
        // Options
        Options options;

        // Robot Base
        ChassisBase &chassis;
        OdomSource &odomSource;

        // Drive Step Variables
        Pose targetPose = Pose();

    private:
        static constexpr double POST_DRIVE_DELAY = 500; // ms
    };
}