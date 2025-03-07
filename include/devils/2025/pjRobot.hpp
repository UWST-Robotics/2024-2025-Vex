#pragma once

#include "../devils.h"
#include "subsystems/ConveyorSystem.hpp"
#include "subsystems/IntakeSystem.hpp"
#include "subsystems/MogoGrabSystem.hpp"
#include "autonomous/autoFactory.hpp"

namespace devils
{
    struct PJRobot : public Robot
    {
        PJRobot()
        {
            imu.calibrate();

            conveyorSensor.setLEDBrightness(100);
            conveyor.useSensor(&conveyorSensor);

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
            imu.calibrate();
            imu.waitUntilCalibrated();

            autoRoutine->run();
        }

        void opcontrol() override
        {
            // Default State
            intakeSystem.setArmPosition(IntakeSystem::BOTTOM_RING);
            mogoGrabber.setMogoGrabbed(false);

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
                bool highArmInput = mainController.get_digital(DIGITAL_X) || mainController.get_digital(DIGITAL_RIGHT);

                bool clawInput = mainController.get_digital_new_press(DIGITAL_R1) || mainController.get_digital_new_press(DIGITAL_R2);
                bool mogoInput = mainController.get_digital_new_press(DIGITAL_L2) || mainController.get_digital_new_press(DIGITAL_L1);

                // Curve Joystick Inputs for improved control
                leftY = JoystickCurve::curve(leftY, 3.0, 0.1, 0.15);
                leftX = JoystickCurve::curve(leftX, 3.0, 0.05, 0.2);
                rightX = JoystickCurve::curve(rightX, 3.0, 0.05, 0.2);
                rightY = JoystickCurve::curve(rightY, 3.0, 0.1, 0.15);

                // Decrease turning speed for improved control
                rightX *= 0.7;

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

                // Conveyor
                conveyor.setMogoGrabbed(mogoGrabber.isMogoGrabbed());
                conveyor.setPickupRing(true);
                conveyor.setArmLowered(false);
                conveyor.moveAutomatic(rightY);
                conveyor.setRingSorting(RingType::NONE);

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

            // Stop all async tasks
            AutoAsyncStep::stopAll();
        }

        // Constants
        static constexpr double DEAD_WHEEL_RADIUS = 1.0; // in
        static constexpr double CONVEYOR_LENGTH = 84.0;  // teeth
        static constexpr double HOOK_INTERVAL = 21.0;    // teeth
        static constexpr double REJECT_OFFSET = 13;      // teeth

        // Hardware
        VEXBridge bridge = VEXBridge(0);

        SmartMotorGroup leftMotors = SmartMotorGroup("LeftMotors", {-1, 2, -3, 4, -5});
        SmartMotorGroup rightMotors = SmartMotorGroup("RightMotors", {6, -7, 8, -9, 10});
        SmartMotorGroup conveyorMotors = SmartMotorGroup("ConveyorMotors", {19, -20});
        SmartMotorGroup intakeArmMotors = SmartMotorGroup("IntakeArmMotors", {17, -18});

        RotationSensor verticalSensor = RotationSensor("VerticalOdom", 13);
        RotationSensor horizontalSensor = RotationSensor("HorizontalOdom", 14);

        OpticalSensor conveyorSensor = OpticalSensor("ConveyorSensor", 12);
        InertialSensor imu = InertialSensor("IMU", 16);
        RotationSensor intakeArmSensor = RotationSensor("IntakeArmSensor", 11);

        ADIPneumatic mogoPneumatic = ADIPneumatic("MogoPneumatic", 1);
        ADIPneumatic intakeClawPneumatic = ADIPneumatic("IntakeClawPneumatic", 2);
        ADIDigitalInput mogoLimitSwitch = ADIDigitalInput("MogoLimitSwitch", 3);

        // Subsystems
        TankChassis chassis = TankChassis(leftMotors, rightMotors);
        ConveyorSystem conveyor = ConveyorSystem(conveyorMotors);
        MogoGrabSystem mogoGrabber = MogoGrabSystem(mogoPneumatic);
        IntakeSystem intakeSystem = IntakeSystem(intakeClawPneumatic, intakeArmMotors, intakeArmSensor);
        PerpendicularSensorOdometry odometry = PerpendicularSensorOdometry(verticalSensor, horizontalSensor, DEAD_WHEEL_RADIUS);

        // Auto
        NTOdom ntOdom = NTOdom("PJ", odometry);
        AutoStepList *autoRoutine = AutoFactory::createBlazeSkillsAuto(chassis, odometry, intakeSystem, conveyor, mogoGrabber);

        // Renderer
        EyesRenderer eyes = EyesRenderer();
    };
}