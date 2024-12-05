#pragma once

#include "../devils.h"
#include "subsystems/ConveyorSystem.hpp"
#include "subsystems/IntakeSystem.hpp"
#include "autonomous/autoFactory.hpp"

namespace devils
{
    /**
     * Represents a Robin robot and all of its subsystems.
     */
    struct RobinRobot : public Robot
    {
        /**
         * Creates a new instance of Blaze.
         */
        RobinRobot()
        {
        }

        void autonomous() override
        {
        }

        void opcontrol() override
        {

            // Loop
            while (true)
            {
                // Take Controller Inputs
                double leftY = mainController.get_analog(ANALOG_LEFT_Y) / 127.0;
                double leftX = mainController.get_analog(ANALOG_LEFT_X) / 127.0;
                double rightX = mainController.get_analog(ANALOG_RIGHT_X) / 127.0;
                double rightY = mainController.get_analog(ANALOG_RIGHT_Y) / 127.0;

                // Curve Joystick Inputs
                leftY = JoystickCurve::curve(leftY, 3.0, 0.1);
                leftX = JoystickCurve::curve(leftX, 3.0, 0.05);
                rightX = JoystickCurve::curve(rightX, 3.0, 0.05);
                rightY = JoystickCurve::curve(rightY, 3.0, 0.1);

                // Move Intake
                intakeMotors.moveVoltage(rightY);

                // Move Chassis
                chassis.move(leftY, leftX * 0.5);

                // Delay to prevent the CPU from being overloaded
                pros::delay(20);
            }
        }

        void disabled() override
        {
            // Stop the robot
            chassis.stop();
        }

        // Hardware
        SmartMotorGroup leftMotors = SmartMotorGroup("LeftMotors", {20});
        SmartMotorGroup rightMotors = SmartMotorGroup("RightMotors", {-19});
        SmartMotorGroup intakeMotors = SmartMotorGroup("IntakeMotors", {-18});

        // Subsystems
        TankChassis chassis = TankChassis(leftMotors, rightMotors);

        // Renderer
        EyesRenderer eyes = EyesRenderer();
    };
}