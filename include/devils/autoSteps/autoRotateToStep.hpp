#pragma once
#include "pros/rtos.hpp"
#include "common/autoStep.hpp"
#include "devils/odom/odomSource.hpp"
#include "devils/chassis/chassisBase.hpp"
#include "devils/utils/math.hpp"
#include "devils/utils/pidController.hpp"
#include "devils/odom/poseVelocityCalculator.hpp"

namespace devils
{
    // Forward Declaration
    class AbsoluteStepConverter;

    /**
     * Represents a rotational step in an autonomous routine.
     */
    class AutoRotateToStep : public IAutoStep
    {
        // Allow the absolute step converter to access private members
        friend class AbsoluteStepConverter;

    public:
        struct Options
        {
            /// @brief The PID parameters to snap to an angle. Uses delta radians as the error.
            PIDParams pidParams = PIDParams{0.1, 0.0, 0.0};

            /// @brief The maximum speed in %
            double maxSpeed = 0.3;

            /// @brief The minimum speed in %
            double minSpeed = 0.1;

            /// @brief The distance to the goal in radians
            double goalDist = 0.015;

            /// @brief The maximum speed of the robot in rad/s
            double goalSpeed = 0.01;

            /// @brief The timeout in ms to allow for the step to complete.
            double timeout = 2500;

            /// @brief Setting this to false will rotate to the absolute angle instead of the minimum distance.
            bool useMinimumDistance = true;

            /// @brief The default options for the rotational step.
            static Options defaultOptions;
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
            Options options = Options::defaultOptions)
            : chassis(chassis),
              odomSource(odomSource),
              targetAngle(targetAngle),
              options(options),
              rotationPID(options.pidParams)
        {
        }

        void doStep() override
        {
            // Reset PID
            rotationPID.reset();

            // Start Time
            double startTime = pros::millis();

            // Control Loop
            while (true)
            {
                // Get Current Pose
                Pose currentPose = odomSource.getPose();
                double currentVelocity = odomSource.getAngularVelocity();

                // Calculate distance to start and target
                double currentAngle = currentPose.rotation;
                double distanceToTarget = angleDiff(targetAngle, currentAngle);

                // Check if we are at the target
                bool isAtGoalPose = fabs(distanceToTarget) < options.goalDist;
                bool isAtGoalVelocity = fabs(currentVelocity) < options.goalSpeed;
                if (isAtGoalPose && isAtGoalVelocity)
                    break;

                // Check if we timed out
                double currentTime = pros::millis();
                if (currentTime - startTime > options.timeout)
                    break;

                // Calculate Speed
                double speed = rotationPID.update(distanceToTarget);
                speed = std::clamp(speed, -options.maxSpeed, options.maxSpeed);        // Clamp to max speed
                speed = std::copysign(std::max(fabs(speed), options.minSpeed), speed); // Clamp to min speed

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
        PIDController rotationPID;

        // Drive Step Variables
        double targetAngle = 0;

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
