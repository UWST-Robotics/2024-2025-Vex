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
            // blazeRoutine->runAsync();
            // pjRoutine->runAsync();
        }

        // Network Tables
        VEXBridge bridge = VEXBridge(0);

        // Dummy Chassis
        DummyChassis blazeChassis = DummyChassis();
        // AutoStepList *blazeRoutine = AutoFactory::createBlazeMatchAuto(blazeChassis, blazeChassis);
        NTOdom blazeOdomNT = NTOdom("Blaze", blazeChassis);

        // Autonomous
        DummyChassis pjChassis = DummyChassis();
        // AutoStepList *pjRoutine = AutoFactory::createPJMatchAuto(pjChassis, pjChassis);
        NTOdom pjOdomNT = NTOdom("PepperJack", pjChassis);

        // Additional Network Objects

        // Renderer
        EyesRenderer eyes = EyesRenderer();
    };
}