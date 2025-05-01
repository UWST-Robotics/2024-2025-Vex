#pragma once

#include "../devils.h"
#include "subsystems/ConveyorSystem.hpp"
#include "subsystems/IntakeSystem.hpp"
#include "subsystems/GoalRushSystem.hpp"
#include "subsystems/MogoGrabSystem.hpp"
#include "subsystems/HornLEDSystem.hpp"

namespace devils
{
    struct BlazeRobot : public Robot
    {
        BlazeRobot()
        {
            imu.calibrate();

            conveyorSensor.setLEDBrightness(100);
            conveyor.useSensor(&conveyorSensor);

            mogoGrabber.useSensor(&mogoRushSensor);

            odometry.useIMU(&imu);
            odometry.runAsync();
        }

        void autonomous() override
        {
            // Default State
            intakeSystem.setArmPosition(IntakeSystem::BOTTOM_RING);
            mogoGrabber.setMogoGrabbed(false);

            // Calibrate IMU
            // imu.calibrate();
            imu.waitUntilCalibrated();

            // autoRoutine->run();
        }

        void opcontrol() override
        {
            // Default State
            intakeSystem.setArmPosition(IntakeSystem::BOTTOM_RING);
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
                bool highArmInput = mainController.get_digital(DIGITAL_X);
                bool mogoArmInput = mainController.get_digital(DIGITAL_DOWN);
                bool neutralStakeDownInput = mainController.get_digital(DIGITAL_RIGHT);

                bool clawInput = mainController.get_digital_new_press(DIGITAL_R1) || mainController.get_digital_new_press(DIGITAL_R2);
                bool mogoInput = mainController.get_digital_new_press(DIGITAL_L2) || mainController.get_digital_new_press(DIGITAL_L1);

                bool goalRushInput = mainController.get_digital_new_press(DIGITAL_LEFT);
                bool togglePTOInput = mainController.get_digital_new_press(DIGITAL_UP);

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

                // Intake Claw
                if (clawInput)
                {
                    // Toggle Claw Grabber
                    bool shouldGrab = !intakeSystem.getClawGrabbed();
                    intakeSystem.setClawGrabbed(shouldGrab);

                    if (!shouldGrab)
                        mainController.rumble("..");
                }

                // Mogo Grab
                if (mogoInput)
                {
                    // Goal-Rush Mode
                    if (goalRushSystem.getExtended())
                    {
                        goalRushSystem.setClamped(!goalRushSystem.getClamped());
                        if (goalRushSystem.getClamped())
                            mainController.rumble(".");
                    }

                    // Rear-Mogo Mode
                    else
                    {
                        mogoGrabber.setMogoGrabbed(!mogoGrabber.isMogoGrabbed());
                        if (mogoGrabber.isMogoGrabbed())
                            mainController.rumble(".");
                    }
                }

                // Goal Rush
                if (goalRushInput)
                {
                    goalRushSystem.setExtended(!goalRushSystem.getExtended());
                    if (goalRushSystem.getExtended())
                        mainController.rumble("...");
                }

                // PTO
                if (togglePTOInput)
                {
                    // Toggle PTO
                    isPTOEnabled = !isPTOEnabled;
                    symmetricControl.resetOffsets();
                    if (isPTOEnabled)
                        mainController.rumble("-");
                }
                ptoPneumatic.setExtended(isPTOEnabled);

                // Conveyor
                conveyor.setMogoGrabbed(mogoGrabber.isMogoGrabbed());
                conveyor.setArmLowered(false);
                conveyor.setPaused(false);
                conveyor.moveAutomatic(rightY);
                conveyor.setRingSorting(RingType::NONE);

                // Move Chassis
                if (isPTOEnabled)
                    // Drive symmetrically
                    symmetricControl.drive(leftY);
                else
                    // Drive normally
                    chassis.move(leftY, leftX);

                // Delay to prevent the CPU from being overloaded
                pros::delay(20);
            }
        }

        void disabled() override
        {
            // Stop autonomous
            AutoStep::stopAll();

            // Release the mogo grabber
            mogoGrabber.setMogoGrabbed(false);
        }

        // Constants
        static constexpr double DEAD_WHEEL_RADIUS = 1.0; // in
        static constexpr double CONVEYOR_LENGTH = 84.0;  // teeth
        static constexpr double HOOK_INTERVAL = 21.0;    // teeth
        static constexpr double REJECT_OFFSET = 13;      // teeth

        // Hardware
        VEXBridge bridge = VEXBridge();

        SmartMotorGroup leftMotors = SmartMotorGroup("LeftMotors", {-1, 2, -3, 4, -5});
        SmartMotorGroup rightMotors = SmartMotorGroup("RightMotors", {6, -7, 8, -9, 10});
        SmartMotorGroup conveyorMotors = SmartMotorGroup("ConveyorMotors", {17, -18});
        SmartMotorGroup intakeArmMotors = SmartMotorGroup("IntakeArmMotors", {11, -12});

        RotationSensor verticalSensor = RotationSensor("VerticalOdom", 15);
        RotationSensor horizontalSensor = RotationSensor("HorizontalOdom", 14);

        OpticalSensor conveyorSensor = OpticalSensor("ConveyorSensor", 19);
        InertialSensor imu = InertialSensor("IMU", 20);

        ADIPneumatic intakeClawPneumatic = ADIPneumatic("IntakeClawPneumatic", 1);
        ADIPneumatic goalRushDeployPneumatic = ADIPneumatic("GoalRushDeployPneumatic", 2);
        ADIPneumatic mogoPneumatic = ADIPneumatic("MogoPneumatic", 3);
        ADIPneumatic ptoPneumatic = ADIPneumatic("PTOPneumatic", 4);
        ADIDigitalInput mogoRushSensor = ADIDigitalInput("MogoRushSensor", -5);
        ADIPneumatic goalRushClampPneumatic = ADIPneumatic("GoalRushClampPneumatic", 6);

        // Subsystems
        TankChassis chassis = TankChassis(leftMotors, rightMotors);
        ConveyorSystem conveyor = ConveyorSystem(conveyorMotors);
        MogoGrabSystem mogoGrabber = MogoGrabSystem(mogoPneumatic);
        IntakeSystem intakeSystem = IntakeSystem(intakeClawPneumatic, intakeArmMotors);
        GoalRushSystem goalRushSystem = GoalRushSystem(goalRushDeployPneumatic, goalRushClampPneumatic, mogoRushSensor);
        PerpendicularSensorOdometry odometry = PerpendicularSensorOdometry(verticalSensor, horizontalSensor, DEAD_WHEEL_RADIUS);
        SymmetricControl symmetricControl = SymmetricControl(leftMotors, rightMotors);

        // Auto
        VBOdom vbOdom = VBOdom("Blaze", odometry);
        // AutoStepList *autoRoutine = AutoFactory::createBlazeSkillsAuto(chassis, odometry, intakeSystem, conveyor, mogoGrabber);
        // AutoStepList *autoRoutine = AutoFactory::createPJMatchAuto(chassis, odometry, intakeSystem, conveyor, mogoGrabber, goalRushSystem, false);
        // AutoStepList *startMacro = AutoFactory::createBlazeStartMacro(chassis, odometry, intakeSystem, conveyor, mogoGrabber);

        RobotAutoOptions autoOptions = RobotAutoOptions();
        std::vector<Routine> routines = {
            {0, "Match 1", true},
            {1, "Match 2", true},
            {2, "Skills 1", false},
            {3, "Skills 2", false}};
        // Renderer
        OptionsRenderer optionsRenderer = OptionsRenderer("Blaze", routines, &autoOptions);
    };
}