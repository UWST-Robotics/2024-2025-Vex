#pragma once

#include "../devils.h"
#include "subsystems/ConveyorSystem.hpp"
#include "subsystems/IntakeSystem.hpp"
#include "subsystems/MogoGrabSystem.hpp"
#include "autonomous/pjMatchAuto.hpp"
#include "autonomous/pjSkillsAuto.hpp"
#include "autonomous/testAuto.hpp"

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
            odometry.setSensorOffsets(verticalSensorOffset, horizontalSensorOffset);
            odometry.runAsync();
        }

        void autonomous() override
        {
            // Default State
            intakeSystem.setArmPosition(IntakeSystem::BOTTOM_RING);
            mogoGrabber.setMogoGrabbed(false);

            // Calibrate IMU
            imu.waitUntilCalibrated();

            // Run Autonomous
            // TestAuto::runB(chassis, odometry);
            bool isBlue = autoOptions.allianceColor == AllianceColor::BLUE_ALLIANCE;
            switch (autoOptions.routine.id)
            {
            case 0:
                PJSkillsAuto::runSkills(chassis, odometry, intakeSystem, conveyor, mogoGrabber);
                break;
            case 1:
                PJMatchAuto::southAuto(chassis, odometry, intakeSystem, conveyor, mogoGrabber, isBlue, true);
                break;
            case 2:
                PJMatchAuto::southAuto(chassis, odometry, intakeSystem, conveyor, mogoGrabber, isBlue, false);
                break;
            }
            // PJMatchAuto::southAuto(chassis, odometry, intakeSystem, conveyor, mogoGrabber);
            // PJSkillsAuto::runSkills(chassis, odometry, intakeSystem, conveyor, mogoGrabber);
        }

        void opcontrol() override
        {
            // Default State
            intakeSystem.setArmPosition(IntakeSystem::INTAKE);
            mogoGrabber.setMogoGrabbed(false);

            // Stop autonomous
            AutoStep::stopAll();

            // PTO
            bool isPTOEnabled = false;

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

                bool colorSortInput = mainController.get_digital(DIGITAL_DOWN) || mainController.get_digital(DIGITAL_RIGHT);

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

                // Slow Mode
                double speedMultiplier = slowInput ? 0.5 : 1.0;

                // Conveyor
                conveyor.setMogoGrabbed(mogoGrabber.isMogoGrabbed());
                conveyor.setRingSorting(colorSortInput ? RingType::BLUE : RingType::NONE);
                conveyor.setArmLowered(intakeSystem.getArmPosition() == IntakeSystem::ArmPosition::BOTTOM_RING); // Always allow the conveyor to move
                conveyor.moveAutomatic(pickupInput ? 1.0 : rightY);

                // Move Chassis
                if (isPTOEnabled)
                {
                    // Drive symmetrically
                    symmetricControl.drive(leftY);
                    symmetricControl.driveHorizontal(leftX, 0.5);
                }
                else
                {
                    // Drive normally
                    chassis.move(leftY * speedMultiplier, combinedX * speedMultiplier);
                }

                // Delay to prevent the CPU from being overloaded
                pros::delay(20);
            }
        }

        void disabled() override
        {
            // Stop the robot
            chassis.stop();

            // Stop autonomous
            AutoStep::stopAll();
        }

        // Constants
        static constexpr double DEAD_WHEEL_RADIUS = 0.991; // in (slightly smaller to account for roller play)
        static constexpr double CONVEYOR_LENGTH = 84.0;    // teeth
        static constexpr double HOOK_INTERVAL = 21.0;      // teeth
        static constexpr double REJECT_OFFSET = 13;        // teeth

        Vector2 verticalSensorOffset = Vector2(-0.5, 0);
        Vector2 horizontalSensorOffset = Vector2(0, 1);

        // Hardware
        VEXBridge bridge = VEXBridge();

        SmartMotorGroup leftMotors = SmartMotorGroup("LeftMotors", {-1, 3, -6, 2, -7});
        SmartMotorGroup rightMotors = SmartMotorGroup("RightMotors", {10, -9, 5, -8, 21});
        SmartMotorGroup conveyorMotors = SmartMotorGroup("ConveyorMotors", {-19, 20});
        SmartMotorGroup intakeArmMotors = SmartMotorGroup("IntakeArmMotors", {-17, 18});

        RotationSensor verticalSensor = RotationSensor("VerticalOdom", 13);
        RotationSensor horizontalSensor = RotationSensor("HorizontalOdom", 16);

        OpticalSensor conveyorSensor = OpticalSensor("ConveyorSensor", 11);
        InertialSensor imu = InertialSensor("IMU", 15);

        ADIPneumatic mogoPneumatic = ADIPneumatic("MogoPneumatic", 1);
        ADIPneumatic intakeClawPneumatic = ADIPneumatic("IntakeClawPneumatic", 2);

        ADIDigitalInput ringSensor = ADIDigitalInput("RingSensor", -4);

        // Subsystems
        TankChassis chassis = TankChassis(leftMotors, rightMotors);
        ConveyorSystem conveyor = ConveyorSystem(conveyorMotors);
        MogoGrabSystem mogoGrabber = MogoGrabSystem(mogoPneumatic);
        IntakeSystem intakeSystem = IntakeSystem(intakeClawPneumatic, intakeArmMotors);
        PerpendicularSensorOdometry odometry = PerpendicularSensorOdometry(verticalSensor, horizontalSensor, DEAD_WHEEL_RADIUS);
        SymmetricControl symmetricControl = SymmetricControl(leftMotors, rightMotors);

        // Auto
        VBOdom vbOdom = VBOdom("PJ", odometry);

        RobotAutoOptions autoOptions = RobotAutoOptions();
        std::vector<Routine> routines = {
            {0, "Skills", false},
            {1, "Match (end center)", true},
            {2, "Match (end side)", true},
        };
        // Renderer
        OptionsRenderer optionsRenderer = OptionsRenderer("PepperJack", routines, &autoOptions);
    };
}