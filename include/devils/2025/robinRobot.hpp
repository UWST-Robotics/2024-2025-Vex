#pragma once

#include "../devils.h"
#include "subsystems/ConveyorSystem.hpp"
#include "subsystems/IntakeSystem.hpp"

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
            imu.setHeadingScale(1.013);

            odometry.useIMU(&imu);
            odometry.setTicksPerRevolution(300);
            odometry.runAsync();
        }

        void autonomous() override
        {
            imu.calibrate();
            imu.waitUntilCalibrated();
            imu.setHeading(M_PI);

            // autoRoutine->run();
        }

        void opcontrol() override
        {
            double steeringControl = 0.5;
            int driveMode = 0;

            // Loop
            while (true)
            {
                // Take Controller Inputs
                double leftY = mainController.get_analog(ANALOG_LEFT_Y) / 127.0;
                double leftX = mainController.get_analog(ANALOG_LEFT_X) / 127.0;
                double rightX = mainController.get_analog(ANALOG_RIGHT_X) / 127.0;
                double rightY = mainController.get_analog(ANALOG_RIGHT_Y) / 127.0;

                bool up = mainController.get_digital_new_press(DIGITAL_UP);
                bool down = mainController.get_digital_new_press(DIGITAL_DOWN);
                bool switchDrive = mainController.get_digital_new_press(DIGITAL_A);
                bool lowerSteer = mainController.get_digital(DIGITAL_R2);

                if (up)
                    steeringControl += 0.05;
                if (down)
                    steeringControl -= 0.05;
                if (up || down)
                    mainController.set_text(0, 0, "Steer: " + std::to_string(steeringControl * 100) + "%");

                if (switchDrive)
                {
                    driveMode = (driveMode + 1) % 2;
                    if (driveMode == 0)
                        mainController.set_text(0, 0, "Split/Arcade   ");
                    if (driveMode == 1)
                        mainController.set_text(0, 0, "Tank           ");
                }

                // Curve Joystick Inputs
                leftY = JoystickCurve::curve(leftY, 3.0, 0.1, 0.15);
                leftX = JoystickCurve::curve(leftX, 4.0, 0.05, 0.2);
                rightX = JoystickCurve::curve(rightX, 4.0, 0.05, 0.2);
                rightY = JoystickCurve::curve(rightY, 3.0, 0.1, 0.15);

                // mainController.set_text(0, 0, std::to_string(lerpedX));

                // Move Chassis
                double steering = leftX;
                if (std::abs(rightX) > std::abs(leftX))
                    steering = rightX;
                if (lowerSteer)
                    steering *= 0.5;

                if (driveMode == 0)
                    chassis.move(leftY, steering * steeringControl);
                if (driveMode == 1)
                    chassis.moveTank(leftY, rightY);

                // Move Conveyor
                // conveyorMotors.moveVoltage(rightY);

                // Delay to prevent the CPU from being overloaded
                pros::delay(20);
            }
        }

        void disabled() override
        {
            // Stop the robot
            chassis.stop();
        }

        // VEXBridge
        // VEXBridge bridge = VEXBridge(0);

        // Hardware
        SmartMotorGroup leftMotors = SmartMotorGroup("LeftMotors", {-6, 7, -8, 9, -10});
        SmartMotorGroup rightMotors = SmartMotorGroup("RightMotors", {16, -17, 18, -19, 20});
        SmartMotorGroup conveyorMotors = SmartMotorGroup("ConveyorMotors", {1, -2});
        InertialSensor imu = InertialSensor("IMU", 15);

        // Subsystems
        TankChassis chassis = TankChassis(leftMotors, rightMotors);

        // Auto
        TankChassisOdom odometry = TankChassisOdom(chassis, 1.375, 11);
        // AutoStepList *autoRoutine = AutoFactory::createBlazeMatchAuto(chassis, odometry);

        // NT
        VBOdom vbOdom = VBOdom("TankOdom", odometry);

        // Renderer
        EyesRenderer eyes = EyesRenderer();
    };
}