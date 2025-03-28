#pragma once

#include "devils/devils.h"
#include "../subsystems/IntakeSystem.hpp"
#include "../subsystems/ConveyorSystem.hpp"
#include "../subsystems/MogoGrabSystem.hpp"

namespace devils
{
    struct TestAuto
    {
        static void run(
            ChassisBase &chassis,
            OdomSource &odometry)
        {
            // PID Params
            PIDParams rotatePID = {0.7, 0.0, 50.0};

            // Default Options
            AutoRotateToStep::Options::defaultOptions = {rotatePID};

            // Initialize
            AutoBuilder pjRoutine = AutoBuilder(chassis, odometry);
            pjRoutine.setPose(-56, 14, 130)->run();
            pjRoutine.pause(1000)->run();

            pjRoutine.driveToTrajectory(-38, -20, 100, true)->run();        // Pickup Goal
            pjRoutine.driveToTrajectory(-60, 0, 110, false, 30, 6)->run();  // Ring 1
            pjRoutine.driveToTrajectory(-44, 48, 60, false, 30, 18)->run(); // Ring 2
            pjRoutine.driveToTrajectory(-24, 48, -60, false, 20)->run();    // Ring 3
            pjRoutine.driveToTrajectory(-48, 24, 150, false, 20, 24)->run();
            pjRoutine.driveToTrajectory(-50, 50, 135, false, 10)->run();
            pjRoutine.driveToTrajectory(-60, 60, 135, false)->run(); // Corner
            pjRoutine.pause(1000)->run();
            pjRoutine.driveToTrajectory(-50, 50, 135, true)->run();
            pjRoutine.rotateTo(315)->run();
            pjRoutine.driveToTrajectory(-60, 60, 315, true)->run(); // Corner
        }
    };
}