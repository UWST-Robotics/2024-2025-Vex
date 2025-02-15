#pragma once

#include "../devils.h"
#include "subsystems/ConveyorSystem.hpp"
#include "subsystems/IntakeSystem.hpp"
#include "autonomous/autoFactory.hpp"

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
            // networkOdom.setSize(15.0, 15.0);

            // Initialize Hardware
            imu.calibrate();
            imu.setHeadingScale(IMU_HEADING_SCALE);

            // Initialize Subsystems
            conveyor.useSensor(&opticalSensor);

            deadWheelOdom.useIMU(&imu);
            deadWheelOdom.runAsync();
        }

        void autonomous() override
        {
            conveyor.runAsync();

            intakeLauncher.extend();
            conveyor.setSortingEnabled(true);

            // imu.calibrate();
            imu.waitUntilCalibrated();
            imu.setHeading(0);

            autoRoutine.doStep();
        }

        void opcontrol() override
        {
            // Stop Autonomous Tasks
            conveyor.stopAsync();

            double intakeSpeed = 1.0;
            intakeLauncher.extend();

            // Game Timer
            gameTimer.start(60000); // 60 seconds

            // Loop
            while (true)
            {
                // Take Controller Inputs
                double leftY = mainController.get_analog(ANALOG_LEFT_Y) / 127.0;
                double leftX = mainController.get_analog(ANALOG_LEFT_X) / 127.0;
                double intakeInput = mainController.get_analog(ANALOG_RIGHT_Y) / 127.0;
                bool grabInput = mainController.get_digital_new_press(DIGITAL_A);
                bool sortingInput = mainController.get_digital(DIGITAL_L1) || mainController.get_digital(DIGITAL_L2);

                // Curve Joystick Inputs
                leftY = JoystickCurve::curve(leftY, 3.0, 0.1);
                leftX = JoystickCurve::curve(leftX, 3.0, 0.05);
                intakeInput = JoystickCurve::curve(intakeInput, 3.0, 0.1);

                // Disable Auto Reject
                conveyor.setSortingEnabled(sortingInput);

                // Move Conveyor/Intake
                conveyor.moveAutomatic(intakeInput * 0.75);
                intake.move(intakeInput);

                // Grab Mogo
                if (grabInput)
                {
                    // Toggle the goal grabber
                    bool isGoalGrabbed = !conveyor.goalGrabbed();
                    conveyor.setGoalGrabbed(isGoalGrabbed);

                    // Controller feedback
                    mainController.rumble(isGoalGrabbed ? "-" : "..");
                }

                // Check Timer
                bool final10 = gameTimer.timeRemaining() < 10000;
                bool isFinished = gameTimer.finished();
                if (final10 && !isFinished)
                    mainController.rumble("..");

                // Print Motor Temps
                mainController.print(0, 0, "Temp: %f", conveyorMotors.getTemperature());

                // Move Chassis
                chassis.move(leftY, leftX);

                // Delay to prevent the CPU from being overloaded
                pros::delay(20);
            }
        }

        void disabled() override
        {
            // Stop the robot
            chassis.stop();
            intakeLauncher.retract();

            // Game Timer
            gameTimer.stop();

            // Tasks
            conveyor.stopAsync();
            // conveyor.setGoalGrabbed(false);
        }

        // Constants
        static constexpr double TICKS_PER_REVOLUTION = 300.0 * (48.0 / 36.0); // ticks
        static constexpr double WHEEL_RADIUS = 1.625;                         // in
        static constexpr double WHEEL_BASE = 12.0;                            // in
        static constexpr double DEAD_WHEEL_RADIUS = 1.0;                      // in
        static constexpr double IMU_HEADING_SCALE = 1.014;                    // %

        // Hardware
        VEXBridge bridge = VEXBridge(5);
        ADIPneumatic grabberPneumatic = ADIPneumatic("GrabberPneumatic", 1);
        ADIPneumatic intakeLauncher = ADIPneumatic("IntakeLauncher", 3);

        SmartMotorGroup leftMotors = SmartMotorGroup("LeftMotors", {11, -12, 18, -20});
        SmartMotorGroup rightMotors = SmartMotorGroup("RightMotors", {-19, 17, 15, -16});
        SmartMotorGroup conveyorMotors = SmartMotorGroup("ConveyorMotors", {3, -4});
        SmartMotorGroup intakeMotors = SmartMotorGroup("IntakeMotors", {10});

        OpticalSensor opticalSensor = OpticalSensor("OpticalSensor", 6);
        RotationSensor verticalSensor = RotationSensor("VerticalOdom", 9);
        RotationSensor horizontalSensor = RotationSensor("HorizontalOdom", 2);
        InertialSensor imu = InertialSensor("IMU", 13);

        // Subsystems
        Timer gameTimer = Timer();
        TankChassis chassis = TankChassis(leftMotors, rightMotors);
        IntakeSystem intake = IntakeSystem(intakeMotors);
        ConveyorSystem conveyor = ConveyorSystem(conveyorMotors, grabberPneumatic);
        // TankChassisOdom chassisOdom = TankChassisOdom(chassis, WHEEL_RADIUS, WHEEL_BASE);
        PerpendicularSensorOdometry deadWheelOdom = PerpendicularSensorOdometry(verticalSensor, horizontalSensor, DEAD_WHEEL_RADIUS);

        // Autonomous Routine
        AutoStepList autoRoutine = AutoFactory::createPJSkillsAuto(chassis, deadWheelOdom, intake, conveyor);

        // Renderer
        EyesRenderer eyes = EyesRenderer();
    };
}