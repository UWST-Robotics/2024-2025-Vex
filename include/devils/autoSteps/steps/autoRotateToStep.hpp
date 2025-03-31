#pragma once
#include "pros/rtos.hpp"
#include "../autoStep.hpp"
#include "../../odom/odomSource.hpp"
#include "../../chassis/chassisBase.hpp"
#include "../../utils/math.hpp"
#include "../../controller/pidController.hpp"
#include "../../odom/poseVelocityCalculator.hpp"

namespace devils
{
    /**
     * Rotates the robot to a specific angle along its center of rotation.
     */
    class AutoRotateToStep : public AutoStep
    {
    public:
        struct Options
        {
            /// @brief The PID parameters to snap to an angle. Uses delta radians as the error.
            PIDController::Options pidParams = {0.1, 0.0, 0.0};

            /// @brief The minimum speed in %
            double minSpeed = 0.1;

            /// @brief The maximum speed in %
            double maxSpeed = 0.6;

            /// @brief The distance to the goal in radians
            double goalDist = 0.015;

            /// @brief The maximum goal speed of the robot in rad/s. (Defaults to no limit)
            double goalSpeed = std::numeric_limits<double>::max();

            /// @brief Setting this to false will rotate to the absolute angle instead of the minimum distance.
            bool useMinimumDistance = true;

            /// @brief The default options for the rotational step.
            static Options defaultOptions;
        };

        /**
         * Rotates the robot to a specific angle along its center of rotation.
         * @param chassis The chassis to control.
         * @param odomSource The odometry source to use.
         * @param targetAngle The angle to rotate to in radians.
         * @param options The options for the rotational step.
         */
        AutoRotateToStep(
            ChassisBase &chassis,
            OdomSource &odomSource,
            double targetAngle,
            Options options = Options::defaultOptions)
            : chassis(chassis),
              odomSource(odomSource),
              targetAngle(targetAngle),
              options(options),
              rotationPID(options.pidParams)
        {
        }

        Options &getOptions()
        {
            return options;
        }

        void onStart() override
        {
            // Reset Finished
            this->isAtGoal = false;

            // Reset PID
            rotationPID.reset();
        }

        void onUpdate() override
        {
            // Get Current Pose
            Pose currentPose = odomSource.getPose();
            double currentVelocity = odomSource.getVelocity().rotation;

            // Calculate distance to start and target
            double currentAngle = currentPose.rotation;
            double distanceToTarget = angleDiff(targetAngle, currentAngle);

            // Check if we are at the goal
            bool isAtGoalPose = fabs(distanceToTarget) < options.goalDist;
            bool isAtGoalVelocity = fabs(currentVelocity) < options.goalSpeed;
            isAtGoal = isAtGoalPose && isAtGoalVelocity;

            // Calculate Speed
            double speed = rotationPID.update(distanceToTarget);
            speed = std::clamp(speed, -options.maxSpeed, options.maxSpeed);        // Clamp to max speed
            speed = std::copysign(std::max(fabs(speed), options.minSpeed), speed); // Clamp to min speed

            // Move Chassis
            chassis.move(0, speed);
        }

        void onStop() override
        {
            // Stop Chassis
            chassis.stop();
        }

        bool checkFinished() override
        {
            return isAtGoal;
        }

    protected:
        // State
        bool isAtGoal = false;

        // Params
        ChassisBase &chassis;
        OdomSource &odomSource;
        PIDController rotationPID;
        double targetAngle = 0;
        Options options;

    private:
        static constexpr double POST_DRIVE_DELAY = 50; // ms

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

    // Initialize Default Options
    AutoRotateToStep::Options AutoRotateToStep::Options::defaultOptions = AutoRotateToStep::Options();
}
