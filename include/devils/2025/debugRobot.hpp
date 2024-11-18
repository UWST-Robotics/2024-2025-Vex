#pragma once

#include "../devils.h"

namespace devils
{
    /**
     * Represents a debugging robot and all of its subsystems.
     * Designed to be used without any physical hardware.
     */
    struct DebugRobot : public Robot
    {
        /**
         * Creates a new instance of the debug robot.
         */
        DebugRobot()
        {
            NetworkTables::Reset();
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
        AutoStepList autoRoutine = AutoStepList({new AutoDriveStep(chassis, chassis, 24.0),
                                                 new AutoRotateToStep(chassis, chassis, M_PI / 2.0)});

        // Additional Network Objects
        NTOdom networkOdom = NTOdom("DummyOdom", chassis);
    };
}