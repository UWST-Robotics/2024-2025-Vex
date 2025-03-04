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
            while (true)
            {
                // int32_t objectCount = visionSensor.getObjectCount();
                // std::cout << "Object Count: " << objectCount << std::endl;
                pros::delay(500);
            }
        }

        // Network Tables
        // VEXBridge bridge = VEXBridge(0);

        // Dummy Chassis
        DummyChassis blazeChassis = DummyChassis();
        // AutoStepList *blazeRoutine = AutoFactory::createTestAuto(blazeChassis);
        // NTOdom blazeOdomNT = NTOdom("Blaze", blazeChassis);
        // AIVisionSensor visionSensor = AIVisionSensor("TestVision", 1);

        // Autonomous
        // DummyChassis pjChassis = DummyChassis();
        // AutoStepList *pjRoutine = AutoFactory::createPJMatchAuto(pjChassis, pjChassis);
        // NTOdom pjOdomNT = NTOdom("PepperJack", pjChassis);

        // Additional Network Objects

        // Renderer
        EyesRenderer eyes = EyesRenderer();
    };
}