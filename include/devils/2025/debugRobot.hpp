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
            blazeRoutine->runAsync();
        }

        // Network Tables
        VEXBridge bridge = VEXBridge(0);

        // Dummy Chassis
        DummyChassis dummyChassis = DummyChassis();
        AutoStepList *blazeRoutine = AutoFactory::createTestAuto(dummyChassis);
        NTOdom dummyOdomNT = NTOdom("DummyOdom", dummyChassis);

        // Renderer
        EyesRenderer eyes = EyesRenderer();
    };
}