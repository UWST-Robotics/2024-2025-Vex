#pragma once
#include <string>
#include "pros/rtos.hpp"
#include "../common/autoStep.hpp"
#include "../../utils/math.hpp"
#include "../../odom/odomSource.hpp"
#include "../../chassis/chassisBase.hpp"
#include "../../utils/pidController.hpp"
#include "../../utils/timer.hpp"
#include "../../odom/poseVelocityCalculator.hpp"

namespace devils
{
    /**
     * Drives the robot linearly to a specific pose.
     * Rotates the robot to face the target pose, disregarding target rotation.
     */
    class AutoDriveToStep : public AutoStep
    {
    public:
        struct Options
        {
            /// @brief The PID parameters for translation. Uses delta inches as the error.
            PIDParams translationPID = PIDParams{0.1, 0.0, 0.0};

            /// @brief The PID parameters for rotation. Uses delta radians as the error.
            PIDParams rotationPID = PIDParams{0.05, 0.0, 0.0};

            /// @brief THe minimum speed in %
            double minSpeed = 0.0;

            /// @brief The maximum speed in %
            double maxSpeed = 0.5;

            /// @brief The maximum final distance to the target in inches
            double goalDist = 6.0;

            /// @brief The maximum final speed of the robot in in/s
            double goalSpeed = 6.0;

            /// @brief The minimum distance from the target to apply rotation. If we are closer than this, we will not rotate to avoid oscillation.
            double minDistanceToRotate = 6.0;

            /// @brief The default options for the drive step.
            static Options defaultOptions;
        };

        /**
         * Drives the robot to a given pose.
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

        void onStart() override
        {
            // Reset PID Controllers
            translationPID.reset();
            rotationPID.reset();
        }

        void onUpdate() override
        {
            // Get Current State
            Pose currentPose = odomSource.getPose();
            Vector2 currentVelocity = odomSource.getVelocity();

            // Calculate distance to start and target
            double distanceToTarget = currentPose.distanceTo(targetPose);

            // Calculate target angle
            double targetAngleRads = std::atan2(
                targetPose.y - currentPose.y,
                targetPose.x - currentPose.x);

            // Calculate Dot Product
            double currentDotTarget = cos(currentPose.rotation) *
                                          (targetPose.x - currentPose.x) +
                                      sin(currentPose.rotation) *
                                          (targetPose.y - currentPose.y);

            // Drive in reverse if the goal is behind us
            if (currentDotTarget < 0)
            {
                distanceToTarget = -distanceToTarget;
                targetAngleRads += M_PI;
            }

            // Calculate Forward Speed
            double speed = translationPID.update(distanceToTarget);
            speed = std::clamp(speed, -options.maxSpeed, options.maxSpeed);

            // Calculate Turn Speed
            double turnSpeed = 0;
            if (std::fabs(distanceToTarget) > options.minDistanceToRotate)
            {
                // Difference in angle
                double angleDiff = Math::angleDiff(targetAngleRads, currentPose.rotation);

                turnSpeed = rotationPID.update(angleDiff);
                turnSpeed = std::clamp(turnSpeed, -options.maxSpeed, options.maxSpeed);
            }

            // Move Chassis
            chassis.move(speed, turnSpeed);
        }

        void onStop() override
        {
            // Stop Chassis
            chassis.stop();
        }

        bool checkFinished() override
        {
            // Get Current State
            Pose currentPose = odomSource.getPose();
            Vector2 currentVelocity = odomSource.getVelocity();

            // Calculate distance to target pose
            double distanceToTarget = currentPose.distanceTo(targetPose);

            // Check if we reached the goal
            bool isAtGoalPose = fabs(distanceToTarget) < options.goalDist;
            bool isAtGoalVelocity = currentVelocity.magnitude() < options.goalSpeed;

            return isAtGoalPose && isAtGoalVelocity;
        }

    protected:
        // Params
        ChassisBase &chassis;
        OdomSource &odomSource;
        PIDController translationPID;
        PIDController rotationPID;
        Pose targetPose = Pose();
        Options options;
    };

    // Define the default options
    AutoDriveToStep::Options AutoDriveToStep::Options::defaultOptions = AutoDriveToStep::Options();
}