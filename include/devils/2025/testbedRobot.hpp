#pragma once

#include "../devils.h"

namespace devils
{
    /**
     * Represents a prototype robot and all of its subsystems.
     */
    struct TestbedRobot : public Robot
    {
        /**
         * Creates a new instance of PepperJack.
         */
        TestbedRobot()
        {
            NetworkTables::Reset();

            // wheelOdom.setTicksPerRevolution(TICKS_PER_REVOLUTION);
            // wheelOdom.runAsync();

            networkOdom.setSize(18.0, 18.0);
        }

        void autonomous() override
        {
            Logger::info("Starting Autonomous");
            autoRoutine.doStep();

            // Stop the robot
            Logger::info("Stopping Autonomous");
            chassis.stop();
        }

        void opcontrol() override
        {
            // Loop
            while (true)
            {
                // Take Controller Inputs
                double leftY = mainController.get_analog(ANALOG_LEFT_Y) / 127.0;
                double leftX = mainController.get_analog(ANALOG_LEFT_X) / 127.0;

                // Curve Joystick Inputs
                leftY = JoystickCurve::curve(leftY, 3.0, 0.1);
                leftX = JoystickCurve::curve(leftX, 3.0, 0.05);

                NetworkTables::UpdateValue("Controls/LeftY", leftY);
                NetworkTables::UpdateValue("Controls/LeftX", leftX);

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

        // Dummy Chassis
        DummyChassis chassis = DummyChassis();

        // Test Autonomous
        AutoRotateStep step1 = AutoRotateStep(chassis, chassis, M_PI / 4.0 + M_PI + M_PI + M_PI + M_PI);
        AutoDriveStep step2 = AutoDriveStep(chassis, chassis, 24.0);
        AutoRotateStep step3 = AutoRotateStep(chassis, chassis, M_PI / 4.0);
        AutoDriveStep step4 = AutoDriveStep(chassis, chassis, 24.0);
        AutoDriveStep step5 = AutoDriveStep(chassis, chassis, -24.0);
        AutoRotateStep step6 = AutoRotateStep(chassis, chassis, -M_PI / 4.0);
        AutoDriveStep step7 = AutoDriveStep(chassis, chassis, -24.0);
        AutoRotateToStep step8 = AutoRotateToStep(chassis, chassis, 0);

        AutoStepList testList = AutoStepList({
            &step1,
            &step2,
            &step3,
            &step4,
            &step5,
            &step6,
            &step7,
            &step8,
        });
        AutoStepList autoRoutine = AutoStepList({
            &testList,
            &testList,
            &testList,
            &testList,
            &testList,
            &testList,
            &testList,
            &testList,
        });

        // Hardware
        RotationSensor rotationSensor = RotationSensor("RotationSensor", 5);

        // Additional Network Objects
        NetworkService &networkService = NetworkService::getInstance();
        NetworkRobotState networkRobotState = NetworkRobotState();
        NetworkOdom networkOdom = NetworkOdom("DummyOdom", chassis);
    };
}