#pragma once

#include "../devils.h"
#include "subsystems/ConveyorSystem.hpp"
#include "subsystems/IntakeSystem.hpp"
#include "subsystems/WackerSystem.hpp"
#include "subsystems/LadyBrownSystem.hpp"
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
            // deadWheelOdomNT.setSize(EXTERIOR_WIDTH, EXTERIOR_HEIGHT);

            // Initialize Hardware
            imu.calibrate();

            // Initialize Subsystems
            Pose initialPose = Pose(-60, 0);
            conveyor.useSensor(&conveyorSensor);
            conveyor.setAutoRejectParams(CONVEYOR_LENGTH, HOOK_INTERVAL, REJECT_OFFSET);

            deadWheelOdom.useIMU(&imu);
            deadWheelOdom.setPose(initialPose);
            deadWheelOdom.runAsync();
        }

        void autonomous() override
        {
            imu.calibrate();
            imu.waitUntilCalibrated();
            imu.setHeading(0);

            conveyor.setSortingEnabled(true);
            conveyor.runAsync();
            ladyBrown.runAsync();

            autoRoutine.doStep();
        }

        void opcontrol() override
        {
            bool isConveyorUp = true;
            bool isConveyorPaused = false;
            bool isWackerDown = false;

            // Stop Tasks
            conveyor.stopAsync();
            ladyBrown.stopAsync();

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
                bool brownUp = mainController.get_digital(DIGITAL_L1);
                bool brownDown = mainController.get_digital(DIGITAL_L2);
                bool wackerInput = mainController.get_digital_new_press(DIGITAL_B);
                bool grabInput = mainController.get_digital_new_press(DIGITAL_A);
                bool conveyorUp = mainController.get_digital_new_press(DIGITAL_UP);
                bool conveyorDown = mainController.get_digital_new_press(DIGITAL_DOWN);

                // Curve Joystick Inputs
                leftY = JoystickCurve::curve(leftY, 3.0, 0.1);
                leftX = JoystickCurve::curve(leftX, 3.0, 0.05);
                rightX = JoystickCurve::curve(rightX, 3.0, 0.05);
                rightY = JoystickCurve::curve(rightY, 3.0, 0.1);

                rightX *= 0.5; // Decrease turning speed for improved control

                // Reverse Intake
                if (intakeOut || !isConveyorUp)
                {
                    intake.move(-1.0);
                }

                // Intake In
                else if (intakeIn)
                {
                    intake.move(1.0);
                    isConveyorPaused = false; // Unpause the conveyor
                }

                // Stop Intake
                else
                {
                    intake.move(0);
                }

                // Lady Brown
                if (brownUp)
                    ladyBrown.raise();
                else if (brownDown)
                    ladyBrown.lower();
                else
                    ladyBrown.idle();

                // Move Conveyor
                if (conveyorUp)
                    isConveyorUp = true;
                else if (conveyorDown)
                    isConveyorUp = false;

                // Unpause if the goal is released
                if (!conveyor.goalGrabbed())
                    isConveyorPaused = false;

                // Disable Auto Reject
                conveyor.setSortingEnabled(true);

                // Pause Conveyor
                if (isConveyorPaused)
                    conveyor.forceMove(0);

                // Run Forward
                else if (isConveyorUp)
                    conveyor.moveAutomatic(1.0);

                // Eject Rings
                else
                    conveyor.moveAutomatic(-0.5);

                // Grab Mogo
                if (grabInput)
                {
                    // Toggle the goal grabber
                    bool isGoalGrabbed = !conveyor.goalGrabbed();
                    conveyor.setGoalGrabbed(isGoalGrabbed);

                    // Controller feedback
                    mainController.rumble(isGoalGrabbed ? "-" : "..");

                    // Pause the conveyor until intake is activated or mogo is released
                    isConveyorPaused = true;
                }

                // Wacker
                if (wackerInput)
                    isWackerDown = !isWackerDown;

                wacker.setExtended(isWackerDown);

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
            wackerPneumatic.retract();
            conveyor.setGoalGrabbed(false);

            // Tasks
            ladyBrown.stopAsync();
            conveyor.stopAsync();
        }

        // Constants
        static constexpr double TICKS_PER_REVOLUTION = 300.0 * (48.0 / 36.0); // ticks
        static constexpr double WHEEL_RADIUS = 1.625;                         // in
        static constexpr double WHEEL_BASE = 12.0;                            // in
        static constexpr double EXTERIOR_WIDTH = 15.0;                        // in
        static constexpr double EXTERIOR_HEIGHT = 15.0;                       // in
        static constexpr double DEAD_WHEEL_RADIUS = 1.0;                      // in
        static constexpr double CONVEYOR_LENGTH = 84.0;                       // teeth
        static constexpr double HOOK_INTERVAL = 21.0;                         // teeth
        static constexpr double REJECT_OFFSET = 13;                           // teeth

        // Hardware
        ADIPneumatic grabberPneumatic = ADIPneumatic("GrabberPneumatic", 1);
        ADIPneumatic wackerPneumatic = ADIPneumatic("Wacker", 2);

        SmartMotorGroup leftMotors = SmartMotorGroup("LeftMotors", {20, -11, 5, -6});
        SmartMotorGroup rightMotors = SmartMotorGroup("RightMotors", {-1, 2, 3, -4});
        SmartMotorGroup conveyorMotors = SmartMotorGroup("ConveyorMotors", {-9, 10});
        SmartMotorGroup intakeMotors = SmartMotorGroup("IntakeMotors", {18});
        SmartMotorGroup ladyBrownMotors = SmartMotorGroup("LadyBrownMotors", {-13});

        RotationSensor verticalSensor = RotationSensor("VerticalOdom", 16);
        RotationSensor horizontalSensor = RotationSensor("HorizontalOdom", 17);
        RotationSensor ladyBrownSensor = RotationSensor("LadyBrownSensor", -12);

        OpticalSensor conveyorSensor = OpticalSensor("ConveyorSensor", 8);
        IMU imu = IMU("IMU", 15);

        // Subsystems
        TankChassis chassis = TankChassis(leftMotors, rightMotors);
        IntakeSystem intake = IntakeSystem(intakeMotors);
        ConveyorSystem conveyor = ConveyorSystem(conveyorMotors, grabberPneumatic);
        WackerSystem wacker = WackerSystem(wackerPneumatic);
        LadyBrownSystem ladyBrown = LadyBrownSystem(ladyBrownMotors, ladyBrownSensor, conveyor);
        // TankChassisOdom chassisOdom = TankChassisOdom(chassis, WHEEL_RADIUS, WHEEL_BASE);
        // TankChassisOdom chassisOdomNoIMU = TankChassisOdom(chassis, WHEEL_RADIUS, WHEEL_BASE);
        PerpendicularSensorOdometry deadWheelOdom = PerpendicularSensorOdometry(verticalSensor, horizontalSensor, DEAD_WHEEL_RADIUS);
        // NTOdom deadWheelOdomNT = NTOdom("DeadWheelOdom", deadWheelOdom);

        // Autonomous
        AutoStepList autoRoutine = AutoFactory::createBlazeAutoRoutine(chassis, deadWheelOdom, intake, conveyor, wacker);

        // Renderer
        EyesRenderer eyes = EyesRenderer();
    };
}