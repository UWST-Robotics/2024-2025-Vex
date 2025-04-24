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
        }

        void opcontrol() override
        {
            VEXBridge::set<double>("TestValue", 0.0);
            while (true)
            {
                auto value = VEXBridge::get<double>("TestValue", 0.0);
                printf("TestValue: %f\n", value);

                pros::delay(1000);
            }
        }

        // Network Tables
        VEXBridge bridge = VEXBridge();

        // Dummy Chassis
        DummyChassis dummyChassis = DummyChassis();
        // AutoStepList *blazeRoutine = AutoFactory::createTestAuto(dummyChassis);
        // VBOdom vbOdom = VBOdom("DummyOdom", dummyChassis);

        // Auto Options
        RobotAutoOptions autoOptions = RobotAutoOptions();
        // Renderer
        OptionsRenderer optionsRenderer = OptionsRenderer({"Hello", "World"}, &autoOptions);
    };
}