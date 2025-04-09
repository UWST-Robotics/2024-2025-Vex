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
        }

        // Network Tables
        VEXBridge bridge = VEXBridge(0);

        // Dummy Chassis
        DummyChassis dummyChassis = DummyChassis();
        // AutoStepList *blazeRoutine = AutoFactory::createTestAuto(dummyChassis);
        VBOdom vbOdom = VBOdom("DummyOdom", dummyChassis);

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