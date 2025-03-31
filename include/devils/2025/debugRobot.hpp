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

       // Auto Options
       RobotAutoOptions autoOptions = RobotAutoOptions();
       // Renderer
       OptionsRenderer optionsRenderer = OptionsRenderer({"Hello", "World", "This", "Is", "A", "Test", "Of", "The", "OptionsRenderer"}, &autoOptions);
    };
}