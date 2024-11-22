#pragma once

#include "../devils.h"
#include "subsystems/ConveyorSystem.hpp"
#include "subsystems/IntakeSystem.hpp"
#include "autoSteps/AutoIntakeStep.hpp"
#include "autoSteps/AutoMogoStep.hpp"

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
            deadWheelOdomNT.setSize(EXTERIOR_WIDTH, EXTERIOR_HEIGHT);

            // Initialize Hardware
            imu.calibrate();

            // Initialize Subsystems
            Pose initialPose = Pose(-60, 0);
            conveyor.useSensor(&conveyorSensor);

            deadWheelOdom.useIMU(&imu);
            deadWheelOdom.setPose(initialPose);
            deadWheelOdom.runAsync();
        }

        void autonomous() override
        {
            conveyor.runAsync();
            intakeLauncher.extend();
            imu.waitUntilCalibrated();

            autoRoutine.doStep();
        }

        void opcontrol() override
        {
            bool shouldIntake = false;
            intakeLauncher.extend();

            // Loop
            while (true)
            {
                // Take Controller Inputs
                double leftY = mainController.get_analog(ANALOG_LEFT_Y) / 127.0;
                double leftX = mainController.get_analog(ANALOG_LEFT_X) / 127.0;
                double rightX = mainController.get_analog(ANALOG_RIGHT_X) / 127.0;
                double rightY = mainController.get_analog(ANALOG_RIGHT_Y) / 127.0;
                bool intakeToggle = mainController.get_digital_new_press(DIGITAL_R1);
                bool intakeOut = mainController.get_digital(DIGITAL_R2);
                bool grabInput = mainController.get_digital_new_press(DIGITAL_A);

                // Curve Joystick Inputs
                leftY = JoystickCurve::curve(leftY, 3.0, 0.1);
                leftX = JoystickCurve::curve(leftX, 3.0, 0.05);
                rightX = JoystickCurve::curve(rightX, 3.0, 0.05);
                rightY = JoystickCurve::curve(rightY, 3.0, 0.1);

                // Move Conveyor/Intake
                if (intakeOut)
                {
                    conveyor.forceMove(-1);
                    intake.move(-0.5);
                }
                else if (shouldIntake)
                {
                    conveyor.moveAutomatic();
                    intake.move(0.5);
                }
                else
                {
                    conveyor.forceMove(0);
                    intake.move(0);
                }

                // Grab Mogo
                if (grabInput)
                    conveyor.setGoalGrabbed(!conveyor.isGoalGrabbed());

                // Intake
                if (intakeToggle)
                    shouldIntake = !shouldIntake;

                // Move Chassis
                chassis.move(leftY, rightX);

                // Delay to prevent the CPU from being overloaded
                pros::delay(20);
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
        static constexpr double EXTERIOR_WIDTH = 15.0;                        // in
        static constexpr double EXTERIOR_HEIGHT = 15.0;                       // in
        static constexpr double DEAD_WHEEL_RADIUS = 1.0;                      // in

        // Hardware
        ADIPneumatic grabberPneumatic = ADIPneumatic("GrabberPneumatic", 1);
        ADIPneumatic intakeLauncher = ADIPneumatic("IntakeLauncher", 2);

        SmartMotorGroup leftMotors = SmartMotorGroup("LeftMotors", {20, -11, 5, -6});
        SmartMotorGroup rightMotors = SmartMotorGroup("RightMotors", {-1, 2, 3, -4});
        SmartMotorGroup conveyorMotors = SmartMotorGroup("ConveyorMotors", {-9, 10});
        SmartMotorGroup intakeMotors = SmartMotorGroup("IntakeMotors", {18});

        RotationSensor verticalSensor = RotationSensor("VerticalOdom", 16);     // TODO: Fix this ID
        RotationSensor horizontalSensor = RotationSensor("HorizontalOdom", 17); // TODO: Fix this ID
        OpticalSensor conveyorSensor = OpticalSensor("ConveyorSensor", 8);
        IMU imu = IMU("IMU", 15);

        // Subsystems
        TankChassis chassis = TankChassis(leftMotors, rightMotors);
        IntakeSystem intake = IntakeSystem(intakeMotors);
        ConveyorSystem conveyor = ConveyorSystem(conveyorMotors, grabberPneumatic);
        // TankChassisOdom chassisOdom = TankChassisOdom(chassis, WHEEL_RADIUS, WHEEL_BASE);
        // TankChassisOdom chassisOdomNoIMU = TankChassisOdom(chassis, WHEEL_RADIUS, WHEEL_BASE);
        PerpendicularSensorOdometry deadWheelOdom = PerpendicularSensorOdometry(verticalSensor, horizontalSensor, DEAD_WHEEL_RADIUS);

        NTOdom deadWheelOdomNT = NTOdom("DeadWheelOdom", deadWheelOdom);

        // Autonomous Constants
        AutoDriveToStep::Options highSpeed = {
            1.0,  // accelDist
            1.0,  // decelDist
            0.8,  // maxSpeed
            0.15, // minSpeed
            2.0,  // rotationGain
            1.0   // goalDist
        };

        AutoDriveToStep::Options slowSpeed = {
            3.0,  // accelDist
            16.0, // decelDist
            0.3,  // maxSpeed
            0.18, // minSpeed
            2.0,  // rotationGain
            0.3   // goalDist
        };

        // Autonomous
        OdomSource &odometry = deadWheelOdom; // <-- Primary Odometry Source
        AutoStepList autoRoutine = AutoStepList({
            new AutoJumpToStep(odometry, -64, 0, 0),

            // Section 1
            new AutoIntakeStep(intake, 0.5),
            new AutoPauseStep(chassis, 500),
            new AutoDriveStep(chassis, odometry, 16.0), // 1
            new AutoRotateToStep(chassis, odometry, M_PI * 0.25),
            new AutoDriveStep(chassis, odometry, 34.0), // 2
            new AutoRotateToStep(chassis, odometry, M_PI * -0.5),
            new AutoDriveStep(chassis, odometry, -24.0),
            new AutoGrabMogoStep(conveyor, true),
            new AutoRotateToStep(chassis, odometry, 0),
            new AutoDriveStep(chassis, odometry, 24.0), // 3
            new AutoRotateToStep(chassis, odometry, M_PI * 0.5),
            new AutoDriveStep(chassis, odometry, 6.0), // 4
            new AutoPauseStep(chassis, 1000),
            new AutoDriveStep(chassis, odometry, -6.0),
            new AutoRotateToStep(chassis, odometry, M_PI),
            new AutoDriveStep(chassis, odometry, 48.0), // 5
            new AutoRotateToStep(chassis, odometry, M_PI * 0.75),
            new AutoDriveStep(chassis, odometry, 12.0), // 6

            new AutoPauseStep(chassis, 1500),
            new AutoDriveStep(chassis, odometry, 3.0),
            new AutoDriveStep(chassis, odometry, -3.0),
            new AutoPauseStep(chassis, 1500),

            new AutoDriveStep(chassis, odometry, -10.0),
            new AutoRotateToStep(chassis, odometry, M_PI * -0.25),
            new AutoDriveStep(chassis, odometry, -10.0),
            new AutoGrabMogoStep(conveyor, false),

            new AutoDriveStep(chassis, odometry, 12.0),
            new AutoRotateToStep(chassis, odometry, 0),
            new AutoDriveStep(chassis, odometry, 48.0),
            new AutoRotateToStep(chassis, odometry, M_PI * 0.75),
            new AutoDriveStep(chassis, odometry, -33.0),
            new AutoGrabMogoStep(conveyor, true),
            new AutoRotateToStep(chassis, odometry, M_PI * 0.5),
            new AutoDriveStep(chassis, odometry, 24.0), // 1
            new AutoRotateToStep(chassis, odometry, 0),
            new AutoDriveStep(chassis, odometry, 24.0), // 2
            new AutoRotateToStep(chassis, odometry, M_PI * -0.5),
            new AutoDriveStep(chassis, odometry, 24.0), // 3
            new AutoRotateToStep(chassis, odometry, M_PI * -0.35),
            new AutoDriveStep(chassis, odometry, 24.0), // 4
            new AutoDriveStep(chassis, odometry, -24.0),
            new AutoRotateToStep(chassis, odometry, M_PI * -0.5),
            new AutoDriveStep(chassis, odometry, -24.0),
            new AutoRotateToStep(chassis, odometry, M_PI * -0.75),
            new AutoDriveStep(chassis, odometry, -10.0, highSpeed),
            new AutoPauseStep(chassis, 1000),
            new AutoDriveStep(chassis, odometry, 8.0, slowSpeed),
            new AutoDriveStep(chassis, odometry, -8.0, highSpeed),
            new AutoGrabMogoStep(conveyor, false),
            new AutoPauseStep(chassis, 1500),
            new AutoDriveStep(chassis, odometry, 24.0),
        });
    };
}