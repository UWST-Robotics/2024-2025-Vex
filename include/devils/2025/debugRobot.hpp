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

        void opcontrol() override
        {
            while (true)
            {
                testValue.set(rand() * 100);

                pros::delay(1000);
            }
        }

        // Dummy Chassis
        // DummyChassis chassis = DummyChassis();

        // Additional Network Objects
        NTSerial serial = NTSerial(1);
        NTValue<double> testValue = NTValue<double>("test", 0.0);

        // Renderer
        // EyesRenderer eyes = EyesRenderer();
    };
}