#pragma once
#include <string>
#include "pros/rtos.hpp"
#include "common/autoStep.hpp"
#include "devils/utils/math.hpp"
#include "devils/odom/odomSource.hpp"
#include "devils/chassis/chassisBase.hpp"
#include "devils/utils/pidController.hpp"
#include "devils/odom/poseVelocityCalculator.hpp"

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
            /// @brief The PID parameters for translation. Uses delta inches as the error.
            PIDParams translationPID = PIDParams{0.1, 0.0, 0.0};

            /// @brief The PID parameters for rotation. Uses delta radians as the error.
            PIDParams rotationPID = PIDParams{0.05, 0.0, 0.0};

            /// @brief The maximum speed in %
            double maxSpeed = 0.4;

            /// @brief The distance to the goal in inches
            double goalDist = 1.5;

            /// @brief The maximum speed of the robot in in/s
            double goalSpeed = 1;

            /// @brief The timeout in ms to allow for the step to complete.
            double timeout = 2000;

            /// @brief The minimum distance from the target to apply rotation
            double minDistanceToRotate = 12.0;

            /// @brief The default options for the drive step.
            static Options defaultOptions;
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
            Options options = Options::defaultOptions)
            : chassis(chassis),
              odomSource(odomSource),
              targetPose(targetPose),
              options(options),
              translationPID(options.translationPID),
              rotationPID(options.rotationPID)
        {
        }

        void doStep() override
        {
            // Calculate Target Pose
            Pose startPose = odomSource.getPose();

            // Reset PID Controllers
            translationPID.reset();
            rotationPID.reset();

            // Start Time
            double startTime = pros::millis();

            // Control Loop
            while (true)
            {
                // Get Current State
                Pose currentPose = odomSource.getPose();
                Vector2 currentVelocity = odomSource.getVelocity();

                // Calculate distance to start and target
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

                // Check if we reached the goal
                bool isAtGoalPose = fabs(distanceToTarget) < options.goalDist;
                bool isAtGoalVelocity = currentVelocity.magnitude() < options.goalSpeed;
                if (isAtGoalPose && isAtGoalVelocity)
                    break;

                // Check if we timed out
                double currentTime = pros::millis();
                if (currentTime - startTime > options.timeout)
                    break;

                // Calculate Speed
                double speed = translationPID.update(distanceToTarget);
                speed = std::clamp(speed, -options.maxSpeed, options.maxSpeed);

                // Calculate Angle
                double targetAngleRads = std::atan2(
                    targetPose.y - currentPose.y,
                    targetPose.x - currentPose.x);

                if (dot < 0)
                    targetAngleRads += M_PI; // Goal point is behind us, target the opposite direction

                double angleDiff = Math::angleDiff(targetAngleRads, currentPose.rotation);

                // Calculate Turn Speed
                double turnSpeed = 0;
                if (std::fabs(distanceToTarget) > options.minDistanceToRotate)
                {
                    turnSpeed = rotationPID.update(angleDiff);
                    turnSpeed = std::clamp(turnSpeed, -options.maxSpeed, options.maxSpeed);
                }

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
        PIDController translationPID;
        PIDController rotationPID;

        // Input
        Pose targetPose = Pose();

    private:
        static constexpr double POST_DRIVE_DELAY = 50; // ms
    };

    // Define the default options
    AutoDriveToStep::Options AutoDriveToStep::Options::defaultOptions = AutoDriveToStep::Options();
}