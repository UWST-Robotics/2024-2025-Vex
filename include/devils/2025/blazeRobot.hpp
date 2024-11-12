#pragma once

#include "../devils.h"
#include "subsystems/ConveyorSystem.hpp"
#include "subsystems/IntakeSystem.hpp"
#include "autoSteps/AutoConveyorStep.hpp"
#include "autoSteps/AutoIntakeStep.hpp"

namespace devils
{
    /**
     * Represents a blaze robot and all of its subsystems.
     */
    struct BlazeRobot : public Robot
    {
        /**
         * Creates a new instance of Blaze.
         */
        BlazeRobot()
        {
            NetworkTables::Reset();
            networkOdom.setSize(15.0, 15.0);

            imu.waitUntilCalibrated();

            wheelOdom.useIMU(imu);
            wheelOdom.setTicksPerRevolution(TICKS_PER_REVOLUTION);
            wheelOdom.runAsync();
        }

        void autonomous() override
        {
            autoRoutine.doStep();
        }

        void opcontrol() override
        {
            double intakeSpeed = 0.5;

            // Loop
            while (true)
            {
                // Take Controller Inputs
                double leftY = mainController.get_analog(ANALOG_LEFT_Y) / 127.0;
                double leftX = mainController.get_analog(ANALOG_LEFT_X) / 127.0;
                double intakeInput = mainController.get_analog(ANALOG_RIGHT_Y) / 127.0;
                bool intakeSpeedUp = mainController.get_digital_new_press(DIGITAL_UP);
                bool intakeSpeedDown = mainController.get_digital_new_press(DIGITAL_DOWN);

                // Curve Joystick Inputs
                leftY = JoystickCurve::curve(leftY, 3.0, 0.1);
                leftX = JoystickCurve::curve(leftX, 3.0, 0.05);
                intakeInput = JoystickCurve::curve(intakeInput, 3.0, 0.1);

                // Move Conveyor/Intkae
                // conveyor.tryMove(intakeInput);
                intake.move(intakeInput);

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
        }

        // V5 Ports
        static constexpr std::initializer_list<int8_t> L_MOTOR_PORTS = {6, 12, -7, -11};
        static constexpr std::initializer_list<int8_t> R_MOTOR_PORTS = {-2, -8, 9, 1};
        static constexpr std::initializer_list<int8_t> INTAKE_PORTS = {5};
        static constexpr uint8_t IMU_PORT = 3;

        // static constexpr std::initializer_list<int8_t> CONVEYOR_PORTS = {-9, 10};
        // static constexpr int8_t GRABBER_PORT = 1;
        static constexpr double TICKS_PER_REVOLUTION = 300.0 * (48.0 / 36.0); // ticks
        static constexpr double WHEEL_RADIUS = 1.625;                         // in
        static constexpr double WHEEL_BASE = 12.0;                            // in

        // Subsystems
        IMU imu = IMU("IMU", IMU_PORT);
        TankChassis chassis = TankChassis("Chassis", L_MOTOR_PORTS, R_MOTOR_PORTS);
        IntakeSystem intake = IntakeSystem(INTAKE_PORTS);
        // ConveyorSystem conveyor = ConveyorSystem(CONVEYOR_PORTS, GRABBER_PORT);

        // Autonomous
        DifferentialWheelOdometry wheelOdom = DifferentialWheelOdometry(chassis, WHEEL_RADIUS, WHEEL_BASE);
        AutoStepList autoRoutine = AutoStepList({
            // Section 1
            new AutoJumpToStep(wheelOdom, -64, -48, 0),
            new AutoDriveStep(chassis, wheelOdom, 18.0),
            new AutoRotateToStep(chassis, wheelOdom, -M_PI),
            new AutoDriveStep(chassis, wheelOdom, -24.0),
            new AutoRotateToStep(chassis, wheelOdom, -M_PI * 0.9),
            new AutoDriveStep(chassis, wheelOdom, 35.0),
            new AutoDriveStep(chassis, wheelOdom, -20.0),
            new AutoRotateToStep(chassis, wheelOdom, M_PI * 0.35),
            new AutoDriveStep(chassis, wheelOdom, 52.0),
            new AutoRotateStep(chassis, wheelOdom, M_PI * 0.1),
            new AutoDriveStep(chassis, wheelOdom, 12.0),
            new AutoRotateStep(chassis, wheelOdom, -M_PI * 0.2),
            new AutoDriveStep(chassis, wheelOdom, 17.0),
            new AutoRotateToStep(chassis, wheelOdom, M_PI * 0.3),
            new AutoDriveStep(chassis, wheelOdom, -95.0),

            // new AutoJumpToStep(wheelOdom, -53, -53, M_PI * 0.3),

        });

        // Debug
        NetworkService &networkService = NetworkService::getInstance();
        NetworkRobotState networkRobotState = NetworkRobotState();
        NetworkOdom networkOdom = NetworkOdom("WheelOdom", wheelOdom);
    };
}