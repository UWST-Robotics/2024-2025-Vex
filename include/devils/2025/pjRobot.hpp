#pragma once

#include "../devils.h"
#include "subsystems/ConveyorSystem.hpp"
#include "subsystems/IntakeSystem.hpp"
#include "autoSteps/AutoIntakeStep.hpp"

namespace devils
{
    /**
     * Represents a PJ robot (15x15) and all of its subsystems.
     */
    struct PJRobot : public Robot
    {
        /**
         * Creates a new instance of PepperJack.
         */
        PJRobot()
        {
            // Initialize NT
            networkOdom.setSize(15.0, 15.0);

            // Initialize Hardware
            imu.calibrate();

            // Initialize Subsystems
            conveyor.useSensor(&opticalSensor);

            deadWheelOdom.useIMU(&imu);
            deadWheelOdom.runAsync();
        }

        void autonomous() override
        {
            conveyor.runAsync();
            intakeLauncher.extend();
            imu.waitUntilCalibrated();
            imu.setHeading(0);

            autoRoutine.doStep();
        }

        void opcontrol() override
        {
            // Stop Autonomous Tasks
            conveyor.stopAsync();

            double intakeSpeed = 0.5;
            intakeLauncher.extend();

            // Loop
            while (true)
            {
                // Take Controller Inputs
                double leftY = mainController.get_analog(ANALOG_LEFT_Y) / 127.0;
                double leftX = mainController.get_analog(ANALOG_LEFT_X) / 127.0;
                double intakeInput = mainController.get_analog(ANALOG_RIGHT_Y) / 127.0;
                bool intakeSpeedUp = mainController.get_digital_new_press(DIGITAL_UP);
                bool intakeSpeedDown = mainController.get_digital_new_press(DIGITAL_DOWN);
                bool grabInput = mainController.get_digital_new_press(DIGITAL_A);

                // Curve Joystick Inputs
                leftY = JoystickCurve::curve(leftY, 3.0, 0.1);
                leftX = JoystickCurve::curve(leftX, 3.0, 0.05);
                intakeInput = JoystickCurve::curve(intakeInput, 3.0, 0.1);

                // Move Conveyor/Intake
                // conveyor.moveAutomatic();
                conveyor.moveAutomatic(intakeInput);
                intake.move(intakeInput);

                // Grab Mogo
                if (grabInput)
                    conveyor.setGoalGrabbed(!conveyor.isGoalGrabbed());

                // Debug Intake Speed
                if (intakeSpeedUp)
                    intakeSpeed += 0.05;
                else if (intakeSpeedDown)
                    intakeSpeed -= 0.05;
                mainController.print(0, 0, "Intake Speed: %f", intakeSpeed);

                // Move Chassis
                chassis.move(leftY, leftX);

                // Delay to prevent the CPU from being overloaded
                pros::delay(10);
            }
        }

        void disabled() override
        {
            // Stop the robot
            chassis.stop();
            intakeLauncher.retract();

            // Tasks
            conveyor.stopAsync();
        }

        // Constants
        static constexpr double TICKS_PER_REVOLUTION = 300.0 * (48.0 / 36.0); // ticks
        static constexpr double WHEEL_RADIUS = 1.625;                         // in
        static constexpr double WHEEL_BASE = 12.0;                            // in
        static constexpr double DEAD_WHEEL_RADIUS = 1.0;                      // in

        // Hardware
        ADIPneumatic grabberPneumatic = ADIPneumatic("GrabberPneumatic", 1);
        ADIPneumatic intakeLauncher = ADIPneumatic("IntakeLauncher", 3);

        SmartMotorGroup leftMotors = SmartMotorGroup("LeftMotors", {11, -12, 18, -20});
        SmartMotorGroup rightMotors = SmartMotorGroup("RightMotors", {-19, 17, 15, -16});
        SmartMotorGroup conveyorMotors = SmartMotorGroup("ConveyorMotors", {3, -4});
        SmartMotorGroup intakeMotors = SmartMotorGroup("IntakeMotors", {10});

        OpticalSensor opticalSensor = OpticalSensor("OpticalSensor", 6);
        RotationSensor verticalSensor = RotationSensor("VerticalOdom", 9);
        RotationSensor horizontalSensor = RotationSensor("HorizontalOdom", 2);
        IMU imu = IMU("IMU", 13);

        // Subsystems
        TankChassis chassis = TankChassis(leftMotors, rightMotors);
        IntakeSystem intake = IntakeSystem(intakeMotors);
        ConveyorSystem conveyor = ConveyorSystem(conveyorMotors, grabberPneumatic);
        // TankChassisOdom chassisOdom = TankChassisOdom(chassis, WHEEL_RADIUS, WHEEL_BASE);
        PerpendicularSensorOdometry deadWheelOdom = PerpendicularSensorOdometry(verticalSensor, horizontalSensor, DEAD_WHEEL_RADIUS);
        NTOdom networkOdom = NTOdom("DeadWheelOdom", deadWheelOdom);

        // Autonomous
        OdomSource &odometry = deadWheelOdom;
        AutoStepList autoRoutine = AutoStepList({
            // Start
            new AutoJumpToStep(odometry, -64, -48, 0),

            // Section 1
            new AutoIntakeStep(intake, 0.5),
            new AutoDriveStep(chassis, odometry, 16.0), // 1
            new AutoRotateToStep(chassis, odometry, M_PI),
            new AutoDriveStep(chassis, odometry, -24.0),
            new AutoGrabMogoStep(conveyor, true),
            new AutoRotateToStep(chassis, odometry, M_PI * 0.5),
            new AutoDriveStep(chassis, odometry, 24.0), // 2
            new AutoRotateToStep(chassis, odometry, M_PI * -0.75),
            new AutoDriveStep(chassis, odometry, 45.0), // 3

            new AutoPauseStep(chassis, 2000),
            new AutoDriveStep(chassis, odometry, 2.0),
            new AutoDriveStep(chassis, odometry, -2.0),
            new AutoPauseStep(chassis, 2000),

            new AutoDriveStep(chassis, odometry, -12.0),
            new AutoRotateToStep(chassis, odometry, M_PI * 0.25),
            new AutoDriveStep(chassis, odometry, -14.0),
            new AutoGrabMogoStep(conveyor, false),

            // Section 2
            new AutoDriveStep(chassis, odometry, 11.0),
            new AutoRotateToStep(chassis, odometry, 0),
            new AutoDriveStep(chassis, odometry, 48.0),
            new AutoRotateToStep(chassis, odometry, M_PI * -0.5),
            new AutoDriveStep(chassis, odometry, 6.0),
            new AutoPauseStep(chassis, 2000),

            // Section 3
            new AutoDriveStep(chassis, odometry, -6.0),
            new AutoRotateToStep(chassis, odometry, M_PI * -0.75),
            new AutoDriveStep(chassis, odometry, -34.0),
            new AutoGrabMogoStep(conveyor, true),
            new AutoRotateToStep(chassis, odometry, M_PI * 0.75),
            new AutoDriveStep(chassis, odometry, 34.0),
            new AutoRotateToStep(chassis, odometry, M_PI * -0.25),
            new AutoGrabMogoStep(conveyor, false),

            // Section 4
            new AutoDriveStep(chassis, odometry, 34.0),
            new AutoRotateToStep(chassis, odometry, M_PI * -0.75),
            new AutoDriveStep(chassis, odometry, -33.0),
            new AutoGrabMogoStep(conveyor, true),
            new AutoRotateToStep(chassis, odometry, M_PI * -0.5),
            new AutoDriveStep(chassis, odometry, 24.0),
            new AutoRotateToStep(chassis, odometry, M_PI * -0.75),
            new AutoDriveStep(chassis, odometry, 32.0),
            new AutoRotateToStep(chassis, odometry, 0),
            new AutoDriveStep(chassis, odometry, 23.0),
            new AutoRotateToStep(chassis, odometry, M_PI * 0.75),
            new AutoDriveStep(chassis, odometry, -16.0),
            new AutoGrabMogoStep(conveyor, false),
            new AutoDriveStep(chassis, odometry, 16.0),
        });
    };
}