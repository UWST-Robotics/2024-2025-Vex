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

        RobotAutoOptions autoOptions = RobotAutoOptions();
        std::vector<Routine> routines = {
            {0, "Match 1", true},
            {1, "Match 2", true},
            {2, "Skills 1", false},
            {3, "Skills 2", false}
        };
        // Renderer
        OptionsRenderer optionsRenderer = OptionsRenderer("Debug", routines, &autoOptions);
    };
}