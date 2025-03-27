#pragma once
#include <string>
#include "pros/rtos.hpp"
#include "../autoStep.hpp"
#include "../../utils/math.hpp"
#include "../../odom/odomSource.hpp"
#include "../../chassis/chassisBase.hpp"
#include "../../trajectory/trajectory.hpp"

namespace devils
{
    /**
     * Drives the robot along a trajectory using ramsete control.
     * See: https://file.tavsys.net/control/controls-engineering-in-frc.pdf
     */
    class AutoRamseteStep : public AutoStep
    {
    public:
        struct Options
        {
            /// @brief The minimum speed in %
            double minSpeed = 0.0;

            /// @brief The maximum speed in %
            double maxSpeed = 0.5;

            /// @brief A proportional constant. Must be greater than 0. Larger values will result in more aggressive control.
            double proportionGain = 1;

            /// @brief A damping coefficient. Must be between 0 and 1. Larger values will result in increased damping.
            double dampingCoefficient = 0.8;

            /// @brief Proportion (P in PID) between translational velocity and motor voltage
            double translationP = 0.82;

            /// @brief Proportion (P in PID) between rotational velocity and motor voltage
            double rotationP = 0.0;

            /// @brief The default options for the drive step.
            static Options defaultOptions;
        };

        /**
         * Drives the robot along a trajectory using ramsete control.
         * @param chassis The chassis to control.
         * @param odomSource The odometry source to use.
         * @param trajectory The trajectory to follow.
         * @param options The options for the drive step.
         */
        AutoRamseteStep(
            ChassisBase &chassis,
            OdomSource &odomSource,
            std::shared_ptr<Trajectory> trajectory,
            Options options = Options::defaultOptions)
            : trajectory(trajectory),
              chassis(chassis),
              odomSource(odomSource),
              options(options)
        {
        }

        void onStart() override
        {
            // Save the start time
            startTime = pros::millis();
        }

        void onUpdate() override
        {
            // Get the current time
            double t = pros::millis() - startTime;
            t /= 1000.0; // Convert to seconds

            // Get current setpoint
            auto setpoint = trajectory->getStateAt(t);

            // Get current position
            auto currentPosition = odomSource.getPose();

            // Calculate error to setpoint
            auto error = setpoint.pose - currentPosition;

            // Calculate local error
            // This is the error relative to the robot's current rotation
            // such that localError.x is aligned with the front of the robot and
            // localError.y is aligned with the side of the robot.
            auto localError = Pose(
                error.x * cos(currentPosition.rotation) + error.y * sin(currentPosition.rotation),
                -error.x * sin(currentPosition.rotation) + error.y * cos(currentPosition.rotation),
                error.rotation);

            VEXBridge::set("errorX", localError.x);
            VEXBridge::set("errorY", localError.y);
            VEXBridge::set("errorR", localError.rotation);
            VEXBridge::set("_poses/setpoint/x", setpoint.pose.x);
            VEXBridge::set("_poses/setpoint/y", setpoint.pose.y);
            VEXBridge::set("_poses/setpoint/rotation", setpoint.pose.rotation);

            // Calculate controller gain
            // k = 2 * zeta * sqrt(w^2 + b * v^2)
            double controllerGain = std::pow(setpoint.angularVelocity, 2) +
                                    options.proportionGain * std::pow(setpoint.velocity, 2);
            controllerGain = 2 * options.dampingCoefficient * std::sqrt(controllerGain);

            // Calculate sinc (sin(x) / x)
            double sinc = sin(localError.rotation) / localError.rotation;
            if (std::isnan(sinc))
                sinc = 1.0; // Handle the case where localError.rotation is 0

            // Calculate translation output
            // v = v_d * cos(e_r) + k * e_x
            double translationOutput = setpoint.velocity * cos(localError.rotation) +
                                       controllerGain * localError.x;

            // Calculate rotation output
            // w = w_d + k * e_r + b * v * sinc(e_r) * e_y
            double rotationOutput = setpoint.angularVelocity +
                                    controllerGain * localError.rotation +
                                    options.proportionGain * setpoint.velocity * sinc * localError.y;

            // Clamp outputs
            translationOutput = Math::deadbandClamp(translationOutput, options.minSpeed, options.maxSpeed);
            rotationOutput = std::clamp(rotationOutput, -options.maxSpeed, options.maxSpeed);

            VEXBridge::set("translationOutput", translationOutput);
            VEXBridge::set("rotationOutput", rotationOutput);
            VEXBridge::set("controllerGain", controllerGain);

            // Get Current Velocity
            auto currentVelocity = odomSource.getVelocity();

            // Calculate the delta velocity and angular velocity
            auto deltaVelocity = currentVelocity.magnitude() - setpoint.velocity;
            auto deltaAngularVelocity = currentVelocity.rotation - setpoint.angularVelocity;

            // Set the chassis output
            chassis.move(
                options.translationP * translationOutput,
                options.rotationP * rotationOutput);
        }

        void onStop() override
        {
            // Stop the chassis
            chassis.stop();
        }

        bool checkFinished() override
        {
            // Get the current time
            auto t = pros::millis() - startTime;

            // Check if the trajectory is finished
            return trajectory->duration() * 1000 <= t;
        }

    protected:
        ChassisBase &chassis;
        OdomSource &odomSource;
        std::shared_ptr<Trajectory> trajectory;
        Options options;

        uint32_t startTime = 0;
    };
}

// Define the default options
devils::AutoRamseteStep::Options devils::AutoRamseteStep::Options::defaultOptions = devils::AutoRamseteStep::Options();