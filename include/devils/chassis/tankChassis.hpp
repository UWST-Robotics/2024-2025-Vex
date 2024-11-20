#pragma once
#include "chassisBase.hpp"
#include "../hardware/smartMotorGroup.hpp"
#include <vector>
#include <iostream>
#include "../utils/logger.hpp"

namespace devils
{
    /**
     * Represents a chassis driven by the differential of two sets of wheels.
     */
    class TankChassis : public ChassisBase
    {
    public:
        /**
         * Creates a new tank chassis.
         * @param name The name of the chassis (for logging purposes)
         * @param leftMotorPorts The ports of the left motors. Negative ports are reversed.
         * @param rightMotorPorts The ports of the right motors. Negative ports are reversed.
         */
        TankChassis(
            SmartMotorGroup leftMotors,
            SmartMotorGroup rightMotors)
            : leftMotors(leftMotors),
              rightMotors(rightMotors)
        {
            // Disable brake mode by default to prevent overheating
            leftMotors.setBrakeMode(false);
            rightMotors.setBrakeMode(false);
        }

        /**
         * Runs the chassis in voltage mode.
         * @param forward The forward speed of the robot from -1 to 1.
         * @param turn The turn speed of the robot from -1 to 1.
         */
        void move(double forward, double turn, double strafe = 0) override
        {
            double fixedForward = std::clamp(forward, -1.0, 1.0);
            double fixedTurn = std::clamp(turn, -1.0, 1.0);
            double fixedStrafe = std::clamp(strafe, -1.0, 1.0);

            fixedForward *= forwardSpeed;
            fixedTurn *= turnSpeed;
            fixedStrafe *= strafeSpeed;

            moveTank(forward + turn, forward - turn);
        }

        /**
         * Runs the chassis in voltage mode with individual control of the left and right sides.
         * @param left The speed to run the left side of the chassis from -1 to 1.
         * @param right The speed to run the right side of the chassis from -1 to 1.
         */
        void moveTank(double left, double right)
        {
            double fixedLeft = std::clamp(left, -1.0, 1.0);
            double fixedRight = std::clamp(right, -1.0, 1.0);

            leftMotors.moveVoltage(fixedLeft);
            rightMotors.moveVoltage(fixedRight);
        }

        /**
         * Gets the left motor group.
         * @return The left motor group.
         */
        SmartMotorGroup &getLeftMotors()
        {
            return leftMotors;
        }

        /**
         * Gets the right motor group.
         * @return The right motor group.
         */
        SmartMotorGroup &getRightMotors()
        {
            return rightMotors;
        }

        /**
         * Forces the chassis to stop.
         */
        void stop() override
        {
            leftMotors.stop();
            rightMotors.stop();
        }

    private:
        SmartMotorGroup &leftMotors;
        SmartMotorGroup &rightMotors;
    };
}