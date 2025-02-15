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
            autoRoutine.doStep();
        }

        // Network Tables
        VEXBridge bridge = VEXBridge(0);

        // Dummy Chassis
        DummyChassis chassis = DummyChassis();
        AutoStepList autoRoutine = AutoFactory::createPJMatchAuto(chassis, chassis);

        // Additional Network Objects
        NTOdom networkOdom = NTOdom("Dummy", chassis);

        // Renderer
        EyesRenderer eyes = EyesRenderer();
    };
}