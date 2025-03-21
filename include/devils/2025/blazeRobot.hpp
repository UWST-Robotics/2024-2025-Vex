#pragma once

#include "../devils.h"
#include "subsystems/ConveyorSystem.hpp"
#include "subsystems/IntakeSystem.hpp"
#include "subsystems/GoalRushSystem.hpp"
#include "subsystems/MogoGrabSystem.hpp"
#include "autonomous/autoFactory.hpp"

namespace devils
{
    struct BlazeRobot : public Robot
    {
        BlazeRobot()
        {
            intakeSystem.setArmPositions(IntakeSystem::ArmPositionAngles{-0.07, -0.18});

            imu.calibrate();

            conveyorSensor.setLEDBrightness(100);
            conveyor.useSensor(&conveyorSensor);

            mogoGrabber.useSensor(&mogoSensor);

            odometry.useIMU(&imu);
            odometry.runAsync();
        }

        void autonomous() override
        {
            // Default State
            intakeSystem.setArmPosition(IntakeSystem::BOTTOM_RING);
            mogoGrabber.setMogoGrabbed(false);
            conveyor.setPickupRing(true); // Always allow the conveyor to pick up rings

            // Calibrate IMU
            // imu.calibrate();
            imu.waitUntilCalibrated();

            autoRoutine->run();
        }

        void opcontrol() override
        {
            // Default State
            intakeSystem.setArmPosition(IntakeSystem::INTAKE);
            mogoGrabber.setMogoGrabbed(false);

            // Start Macro
            // imu.waitUntilCalibrated();
            // startMacro->run();
            AutoAsyncStep::stopAll();

            // Loop
            while (true)
            {
                // Take Controller Inputs
                double leftY = mainController.get_analog(ANALOG_LEFT_Y) / 127.0;
                double leftX = mainController.get_analog(ANALOG_LEFT_X) / 127.0;
                double rightX = mainController.get_analog(ANALOG_RIGHT_X) / 127.0;
                double rightY = mainController.get_analog(ANALOG_RIGHT_Y) / 127.0;

                bool lowArmInput = mainController.get_digital(DIGITAL_B);
                bool midArmInput = mainController.get_digital(DIGITAL_A) || mainController.get_digital(DIGITAL_Y);
                bool highArmInput = mainController.get_digital(DIGITAL_X) || mainController.get_digital(DIGITAL_L1);

                bool pickupInput = mainController.get_digital(DIGITAL_R2);

                bool clawInput = mainController.get_digital_new_press(DIGITAL_R1);
                bool mogoInput = mainController.get_digital_new_press(DIGITAL_L2);
                bool slowInput = false; // mainController.get_digital(DIGITAL_L1);

                bool goalRushInput = mainController.get_digital_new_press(DIGITAL_LEFT);

                // Curve Joystick Inputs
                leftY = JoystickCurve::curve(leftY, 3.0, 0.1, 0.15);
                leftX = JoystickCurve::curve(leftX, 3.0, 0.05, 0.2);
                rightX = JoystickCurve::curve(rightX, 3.0, 0.1, 0.2);
                rightY = JoystickCurve::curve(rightY, 3.0, 0.1, 0.15, 0.8);

                // Decrease turning speed for improved control
                rightX *= 0.5;

                // Combine Left and Right X Joystick Inputs
                double combinedX = JoystickCurve::combine(leftX, rightX);

                // Intake Arm
                if (lowArmInput)
                    intakeSystem.setArmPosition(IntakeSystem::BOTTOM_RING);
                else if (midArmInput)
                    intakeSystem.setArmPosition(IntakeSystem::ALLIANCE_STAKE);
                else if (highArmInput)
                    intakeSystem.setArmPosition(IntakeSystem::NEUTRAL_STAKE);
                else
                    intakeSystem.setArmPosition(IntakeSystem::INTAKE);
                intakeSystem.moveArmToPosition();
                intakeSystem.disableSpeedClamp(lowArmInput);

                // Intake Claw
                if (clawInput)
                {
                    // Toggle Claw Grabber
                    bool shouldGrab = !intakeSystem.getClawGrabbed();
                    intakeSystem.setClawGrabbed(shouldGrab);

                    if (!shouldGrab)
                        mainController.rumble("..");
                }

                // Mogo
                if (mogoInput)
                {
                    // Toggle Mogo Grabber
                    bool shouldGrabGoal = !mogoGrabber.isMogoGrabbed();
                    mogoGrabber.setMogoGrabbed(shouldGrabGoal);

                    if (!shouldGrabGoal)
                        mainController.rumble(".");
                }

                // Goal Rush
                if (goalRushInput)
                {
                    // Toggle Goal Rush
                    bool shouldRush = !goalRushSystem.isGoalRushExtended();
                    goalRushSystem.setGoalRushExtended(shouldRush);

                    if (shouldRush)
                        mainController.rumble("...");
                }

                // Slow Mode
                double speedMultiplier = slowInput ? 0.5 : 1.0;

                // Conveyor
                conveyor.setMogoGrabbed(mogoGrabber.isMogoGrabbed());
                conveyor.setPickupRing(true); // Always allow the conveyor to pick up rings
                conveyor.setRingSorting(RingType::NONE);
                conveyor.setArmLowered(intakeSystem.getArmPosition() == IntakeSystem::ArmPosition::BOTTOM_RING); // Always allow the conveyor to move
                conveyor.moveAutomatic(pickupInput ? 1.0 : rightY);

                // Move Chassis
                chassis.move(leftY * speedMultiplier, combinedX * speedMultiplier);

                // Delay to prevent the CPU from being overloaded
                pros::delay(20);
            }
        }

        void disabled() override
        {
            // Stop autonomous
            AutoAsyncStep::stopAll();
            autoRoutine->stop();

            // Stop the robot
            chassis.stop();

            mogoGrabber.setMogoGrabbed(false);
        }

        // Constants
        static constexpr double DEAD_WHEEL_RADIUS = 1.0; // in
        static constexpr double CONVEYOR_LENGTH = 84.0;  // teeth
        static constexpr double HOOK_INTERVAL = 21.0;    // teeth
        static constexpr double REJECT_OFFSET = 13;      // teeth

        // Hardware
        // VEXBridge bridge = VEXBridge(0);

        SmartMotorGroup leftMotors = SmartMotorGroup("LeftMotors", {-1, 2, -3, 4, -5});
        SmartMotorGroup rightMotors = SmartMotorGroup("RightMotors", {6, -7, 8, -9, 10});
        SmartMotorGroup conveyorMotors = SmartMotorGroup("ConveyorMotors", {-19, 20});
        SmartMotorGroup intakeArmMotors = SmartMotorGroup("IntakeArmMotors", {17, -18});

        RotationSensor verticalSensor = RotationSensor("VerticalOdom", 13);
        RotationSensor horizontalSensor = RotationSensor("HorizontalOdom", 14);

        OpticalSensor conveyorSensor = OpticalSensor("ConveyorSensor", 11);
        InertialSensor imu = InertialSensor("IMU", 16);
        RotationSensor intakeArmSensor = RotationSensor("IntakeArmSensor", 12);

        ADIPneumatic intakeClawPneumatic = ADIPneumatic("IntakeClawPneumatic", 1);
        ADIPneumatic mogoPneumatic = ADIPneumatic("MogoPneumatic", 2);
        ADIDigitalInput mogoSensor = ADIDigitalInput("MogoSensor", -3);
        ADIPneumatic goalRushPneumatic = ADIPneumatic("GoalRushPneumatic", 4);

        // Subsystems
        TankChassis chassis = TankChassis(leftMotors, rightMotors);
        ConveyorSystem conveyor = ConveyorSystem(conveyorMotors);
        MogoGrabSystem mogoGrabber = MogoGrabSystem(mogoPneumatic);
        IntakeSystem intakeSystem = IntakeSystem(intakeClawPneumatic, intakeArmMotors, intakeArmSensor);
        GoalRushSystem goalRushSystem = GoalRushSystem(goalRushPneumatic);
        PerpendicularSensorOdometry odometry = PerpendicularSensorOdometry(verticalSensor, horizontalSensor, DEAD_WHEEL_RADIUS);

        // Auto
        NTOdom ntOdom = NTOdom("Blaze", odometry);
        // AutoStepList *autoRoutine = AutoFactory::createBlazeSkillsAuto(chassis, odometry, intakeSystem, conveyor, mogoGrabber);
        AutoStepList *autoRoutine = AutoFactory::createPJMatchAuto(chassis, odometry, intakeSystem, conveyor, mogoGrabber, goalRushSystem, false);
        AutoStepList *startMacro = AutoFactory::createBlazeStartMacro(chassis, odometry, intakeSystem, conveyor, mogoGrabber);

        // Renderer
        EyesRenderer eyes = EyesRenderer();
    };
}