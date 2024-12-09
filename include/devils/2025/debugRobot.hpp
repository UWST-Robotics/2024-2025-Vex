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
            // Reset Network Tables
            // networkOdom.setSize(18.0, 18.0);
        }

        void autonomous() override
        {
            chassis.stop();
        }

        void opcontrol() override
        {
            autonomous();
        }

        void disabled() override
        {
            // Stop the robot
            chassis.stop();
        }

        // Dummy Chassis
        DummyChassis chassis = DummyChassis();

        // Additional Network Objects
        // NTOdom networkOdom = NTOdom("DummyOdom", chassis);
        // NTPath networkPath = NTPath("TestPath", path);

        // Renderer
        EyesRenderer eyes = EyesRenderer();
    };
}