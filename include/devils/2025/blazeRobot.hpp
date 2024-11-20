#pragma once

#include "../devils.h"
#include "subsystems/ConveyorSystem.hpp"
#include "subsystems/IntakeSystem.hpp"
#include "autoSteps/AutoIntakeStep.hpp"

namespace devils
{
    /**
     * Represents a Blaze robot (24x24) and all of its subsystems.
     */
    struct BlazeRobot : public Robot
    {
        /**
         * Creates a new instance of Blaze.
         */
        BlazeRobot()
        {
            // Initialize NT
            NetworkTables::reset();

            // Initialize Hardware
            imu.calibrate();

            // Initialize Subsystems
            conveyor.useSensor(&conveyorSensor);

            deadWheelOdom.useIMU(&imu);
            deadWheelOdom.runAsync();

            chassisOdom.useIMU(&imu);
            chassisOdom.setTicksPerRevolution(TICKS_PER_REVOLUTION);
            chassisOdom.runAsync();

            chassisOdomNoIMU.setTicksPerRevolution(TICKS_PER_REVOLUTION);
            chassisOdomNoIMU.runAsync();

            chassisOdomNT.setSize(15.0, 15.0);
            chassisOdomNoIMUNT.setSize(15.0, 15.0);
            deadWheelOdomNT.setSize(15.0, 15.0);
        }

        void autonomous() override
        {
            intakeLauncher.extend();
            imu.waitUntilCalibrated();
            autoRoutine.doStep();
        }

        void opcontrol() override
        {
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
                conveyor.moveAutomatic();
                intake.move(intakeInput);

                // Grab Mogo
                if (grabInput)
                {
                    if (conveyor.isGoalGrabbed())
                        conveyor.releaseGoal();
                    else
                        conveyor.grabGoal();
                }

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
        }

        // Constants
        static constexpr double TICKS_PER_REVOLUTION = 300.0 * (48.0 / 36.0); // ticks
        static constexpr double WHEEL_RADIUS = 1.625;                         // in
        static constexpr double WHEEL_BASE = 12.0;                            // in
        static constexpr double DEAD_WHEEL_RADIUS = 1.0;                      // in

        // Hardware
        ADIPneumatic grabberPneumatic = ADIPneumatic("GrabberPneumatic", 1);
        ADIPneumatic intakeLauncher = ADIPneumatic("IntakeLauncher", 2);

        SmartMotorGroup leftMotors = SmartMotorGroup("LeftMotors", {20, -11, 5, -6});
        SmartMotorGroup rightMotors = SmartMotorGroup("RightMotors", {-1, 2, 3, -4});
        SmartMotorGroup conveyorMotors = SmartMotorGroup("ConveyorMotors", {-9, 10});
        SmartMotorGroup intakeMotors = SmartMotorGroup("IntakeMotors", {18});

        RotationSensor verticalSensor = RotationSensor("VerticalOdom", 13);     // TODO: Fix this ID
        RotationSensor horizontalSensor = RotationSensor("HorizontalOdom", 14); // TODO: Fix this ID
        OpticalSensor conveyorSensor = OpticalSensor("ConveyorSensor", 8);
        IMU imu = IMU("IMU", 15);

        // Subsystems
        TankChassis chassis = TankChassis(leftMotors, rightMotors);
        IntakeSystem intake = IntakeSystem(intakeMotors);
        ConveyorSystem conveyor = ConveyorSystem(conveyorMotors, grabberPneumatic);
        TankChassisOdom chassisOdom = TankChassisOdom(chassis, WHEEL_RADIUS, WHEEL_BASE);
        TankChassisOdom chassisOdomNoIMU = TankChassisOdom(chassis, WHEEL_RADIUS, WHEEL_BASE);
        PerpendicularSensorOdometry deadWheelOdom = PerpendicularSensorOdometry(verticalSensor, horizontalSensor, WHEEL_RADIUS);

        NTOdom chassisOdomNT = NTOdom("ChassisOdom", chassisOdom);
        NTOdom chassisOdomNoIMUNT = NTOdom("ChassisOdom_NoIMU", chassisOdomNoIMU);
        NTOdom deadWheelOdomNT = NTOdom("DeadWheelOdom", deadWheelOdom);

        // Autonomous
        OdomSource &odometry = deadWheelOdom; // <-- Primary Odometry Source
        AutoStepList autoRoutine = AutoStepList({

            // Section 1
            new AutoIntakeStep(intake, 0.5),
            new AutoPauseStep(chassis, 500),
            new AutoDriveStep(chassis, odometry, 12.0),
            new AutoRotateToStep(chassis, odometry, M_PI * 0.5),
            new AutoDriveStep(chassis, odometry, 48.0),
            new AutoRotateToStep(chassis, odometry, M_PI),
            new AutoDriveStep(chassis, odometry, -24.0),
            // Pickup Mogo
            // Score Ring
            new AutoRotateToStep(chassis, odometry, M_PI * -0.5),
            new AutoDriveStep(chassis, odometry, 24.0),
            // Score Ring
            new AutoRotateToStep(chassis, odometry, M_PI * 0.25),
            new AutoDriveStep(chassis, odometry, 34.0),
            // Score Ring
            new AutoRotateToStep(chassis, odometry, M_PI * 0.5),
            new AutoDriveStep(chassis, odometry, 6.0),
            // Score Ring
            new AutoRotateToStep(chassis, odometry, M_PI),
            new AutoDriveStep(chassis, odometry, 50.0),
            // Score Ring
            new AutoDriveStep(chassis, odometry, -10.0),
            new AutoRotateToStep(chassis, odometry, M_PI * 0.1),
            new AutoDriveStep(chassis, odometry, -10.0),
            // Drop Mogo

            // Section 3
            new AutoDriveStep(chassis, odometry, 84.0),
            new AutoRotateToStep(chassis, odometry, M_PI * 0.5),
            new AutoDriveStep(chassis, odometry, -25.0),
            // Pickup Mogo
            // Score Ring
            new AutoRotateToStep(chassis, odometry, 0),
            new AutoDriveStep(chassis, odometry, 24.0),
            // Score Ring
            new AutoRotateToStep(chassis, odometry, M_PI * 0.5),
            new AutoDriveStep(chassis, odometry, 24.0),
            // Score Ring
            new AutoRotateToStep(chassis, odometry, M_PI * -0.75),
            new AutoDriveStep(chassis, odometry, -12.0),
            // Drop Mogo

            // Section 4
            new AutoDriveStep(chassis, odometry, 40.0),
            new AutoRotateToStep(chassis, odometry, M_PI * 0.25),
            new AutoDriveStep(chassis, odometry, 34.0),
            new AutoRotateToStep(chassis, odometry, M_PI),
            new AutoDriveStep(chassis, odometry, -5.0),
            // Score Rings
            new AutoPauseStep(chassis, 2000),
            new AutoDriveStep(chassis, odometry, 10.0),
        });
    };
}