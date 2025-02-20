#pragma once

#include "../devils.h"
#include "subsystems/ConveyorSystem.hpp"
#include "subsystems/IntakeSystem.hpp"
#include "subsystems/MogoGrabSystem.hpp"
#include "autonomous/autoFactory.hpp"

namespace devils
{
    struct BlazeRobot : public Robot
    {
        BlazeRobot()
        {
            // Initialize Hardware
            imu.calibrate();

            // Initialize Subsystems
            // conveyor.setAutoRejectParams(CONVEYOR_LENGTH, HOOK_INTERVAL, REJECT_OFFSET);
            conveyor.useSensor(&conveyorSensor);

            mogoGrabber.useLimitSwitch(&mogoLimitSwitch);

            deadWheelOdom.useIMU(&imu);
            deadWheelOdom.runAsync();
        }

        void autonomous() override
        {
        }

        void opcontrol() override
        {
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
                bool highArmInput = mainController.get_digital(DIGITAL_X);

                bool clawInput = mainController.get_digital(DIGITAL_L1) || mainController.get_digital(DIGITAL_L2);
                bool mogoInput = mainController.get_digital_new_press(DIGITAL_R2);
                bool slowInput = mainController.get_digital(DIGITAL_R1);

                // Curve Joystick Inputs
                leftY = JoystickCurve::curve(leftY, 3.0, 0.1, 0.15);
                leftX = JoystickCurve::curve(leftX, 3.0, 0.05, 0.2);
                rightX = JoystickCurve::curve(rightX, 3.0, 0.05, 0.2);
                rightY = JoystickCurve::curve(rightY, 3.0, 0.1, 0.15);

                // Decrease turning speed for improved control
                rightX *= 0.5;

                // Combine Left and Right X Joystick Inputs
                double combinedX = JoystickCurve::combine(leftX, rightX);

                // Intake Arm
                if (lowArmInput)
                    intakeSystem.moveArmToPosition(IntakeSystem::BOTTOM_RING);
                else if (midArmInput)
                    intakeSystem.moveArmToPosition(IntakeSystem::ALLIANCE_STAKE);
                else if (highArmInput)
                    intakeSystem.moveArmToPosition(IntakeSystem::NEUTRAL_STAKE);

                // Intake Claw
                intakeSystem.setClawGrabbed(clawInput);

                // Mogo
                if (mogoInput)
                {
                    // Toggle Mogo Grabber
                    bool shouldGrabGoal = !mogoGrabber.isMogoGrabbed();
                    mogoGrabber.setMogoGrabbed(shouldGrabGoal);
                }

                // Slow Mode
                double speedMultiplier = slowInput ? 0.5 : 1.0;

                // Conveyor
                conveyor.moveAutomatic(rightY);

                // Move Chassis
                chassis.move(leftY * speedMultiplier, combinedX * speedMultiplier);

                // Delay to prevent the CPU from being overloaded
                pros::delay(20);
            }
        }

        void disabled() override
        {
            // Stop the robot
            chassis.stop();

            // Stop all async steps
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
        SmartMotorGroup intakeArmMotors = SmartMotorGroup("IntakeArmMotors", {-17, 18});

        RotationSensor verticalSensor = RotationSensor("VerticalOdom", 16);
        RotationSensor horizontalSensor = RotationSensor("HorizontalOdom", 17);

        InertialSensor imu = InertialSensor("IMU", 15);

        OpticalSensor conveyorSensor = OpticalSensor("ConveyorSensor", 14);

        ADIPneumatic mogoPneumatic = ADIPneumatic("MogoPneumatic", 1);
        ADIPneumatic intakeClawPneumatic = ADIPneumatic("IntakeClawPneumatic", 2);
        ADIDigitalInput mogoLimitSwitch = ADIDigitalInput("MogoLimitSwitch", 3);

        // Subsystems
        TankChassis chassis = TankChassis(leftMotors, rightMotors);
        ConveyorSystem conveyor = ConveyorSystem(conveyorMotors);
        MogoGrabSystem mogoGrabber = MogoGrabSystem(mogoPneumatic);
        IntakeSystem intakeSystem = IntakeSystem(intakeClawPneumatic, intakeArmMotors);
        PerpendicularSensorOdometry deadWheelOdom = PerpendicularSensorOdometry(verticalSensor, horizontalSensor, DEAD_WHEEL_RADIUS);

        // Renderer
        EyesRenderer eyes = EyesRenderer();
    };
}