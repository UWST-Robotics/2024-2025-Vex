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
            // Reset Network Tables
            NetworkTables::reset();
            networkOdom.setSize(15.0, 15.0);

            // Configure Subsystems
            conveyor.useSensor(&opticalSensor);

            // Calibrate IMU
            imu.calibrate();

            // Setup Odometry
            odometry.useIMU(&imu);
            odometry.setTicksPerRevolution(TICKS_PER_REVOLUTION);
            odometry.runAsync();
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

                // Move Conveyor/Intkae
                conveyor.moveAutomatic();
                // conveyor.forceMove(intakeInput);
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

        // V5 Ports
        static constexpr std::initializer_list<int8_t> L_MOTOR_PORTS = {20, -11, 5, -6};
        static constexpr std::initializer_list<int8_t> R_MOTOR_PORTS = {-1, 2, 3, -4};
        static constexpr std::initializer_list<int8_t> INTAKE_PORTS = {18};
        static constexpr std::initializer_list<int8_t> CONVEYOR_PORTS = {-9, 10};
        static constexpr uint8_t IMU_PORT = 15;
        static constexpr uint8_t OPTICAL_SENSOR_PORT = 8;

        static constexpr int8_t GRABBER_PORT = 1;
        static constexpr int8_t INTAKE_LAUNCHER_PORT = 2;

        static constexpr double TICKS_PER_REVOLUTION = 300.0 * (48.0 / 36.0); // ticks
        static constexpr double WHEEL_RADIUS = 1.625;                         // in
        static constexpr double WHEEL_BASE = 12.0;                            // in

        // Subsystems
        ADIPneumatic intakeLauncher = ADIPneumatic("IntakeLauncher", INTAKE_LAUNCHER_PORT);
        IMU imu = IMU("IMU", IMU_PORT);
        TankChassis chassis = TankChassis("Chassis", L_MOTOR_PORTS, R_MOTOR_PORTS);
        IntakeSystem intake = IntakeSystem(INTAKE_PORTS);
        ConveyorSystem conveyor = ConveyorSystem(CONVEYOR_PORTS, GRABBER_PORT);
        OpticalSensor opticalSensor = OpticalSensor("OpticalSensor", OPTICAL_SENSOR_PORT);

        // Autonomous
        TankChassisOdom odometry = TankChassisOdom(chassis, WHEEL_RADIUS, WHEEL_BASE);

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

        // Debug
        NTOdom networkOdom = NTOdom("Odometry", odometry);
    };
}