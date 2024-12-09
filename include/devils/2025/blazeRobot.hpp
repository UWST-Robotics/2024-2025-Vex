#pragma once

#include "../devils.h"
#include "subsystems/ConveyorSystem.hpp"
#include "subsystems/IntakeSystem.hpp"
#include "autonomous/autoFactory.hpp"

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
            intakeLauncher.extend();

            bool isConveyorUp = true;
            bool isConveyorPaused = false;

            // Loop
            while (true)
            {
                // Take Controller Inputs
                double leftY = mainController.get_analog(ANALOG_LEFT_Y) / 127.0;
                double leftX = mainController.get_analog(ANALOG_LEFT_X) / 127.0;
                double rightX = mainController.get_analog(ANALOG_RIGHT_X) / 127.0;
                double rightY = mainController.get_analog(ANALOG_RIGHT_Y) / 127.0;
                bool intakeIn = mainController.get_digital(DIGITAL_R1);
                bool intakeOut = mainController.get_digital(DIGITAL_R2);
                bool grabInput = mainController.get_digital_new_press(DIGITAL_A);
                bool conveyorUp = mainController.get_digital_new_press(DIGITAL_UP);
                bool conveyorDown = mainController.get_digital_new_press(DIGITAL_DOWN);

                // Curve Joystick Inputs
                leftY = JoystickCurve::curve(leftY, 3.0, 0.1);
                leftX = JoystickCurve::curve(leftX, 3.0, 0.05);
                rightX = JoystickCurve::curve(rightX, 3.0, 0.05);
                rightY = JoystickCurve::curve(rightY, 3.0, 0.1);

                rightX *= 0.5;

                // Move Intake
                if (intakeOut || !isConveyorUp)
                {
                    intake.move(-1.0);
                }
                else if (intakeIn)
                {
                    intake.move(1.0);
                    isConveyorPaused = false;
                }
                else
                {
                    intake.move(0);
                }

                // Move Conveyor
                if (conveyorUp)
                    isConveyorUp = true;
                else if (conveyorDown)
                    isConveyorUp = false;

                // Unpause if the goal is released
                if (!conveyor.isGoalGrabbed())
                    isConveyorPaused = false;

                if (isConveyorPaused)
                {
                    conveyor.forceMove(0);
                }
                else if (isConveyorUp)
                {
                    conveyor.moveAutomatic(0.85);
                }
                else
                {
                    conveyor.moveAutomatic(-0.5);
                }

                // Grab Mogo
                if (grabInput)
                {
                    bool isGoalGrabbed = !conveyor.isGoalGrabbed();
                    conveyor.setGoalGrabbed(isGoalGrabbed);
                    mainController.rumble(isGoalGrabbed ? "-" : "..");
                    isConveyorPaused = true;
                }

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
            conveyor.setGoalGrabbed(false);
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

        // Autonomous
        AutoStepList autoRoutine = AutoFactory::createBlazeAutoRoutine(chassis, deadWheelOdom, intake, conveyor);

        // Renderer
        EyesRenderer eyes = EyesRenderer();
    };
}