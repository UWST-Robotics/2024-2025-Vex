#pragma once

#include "devils/devils.h"
#include "../subsystems/IntakeSystem.hpp"
#include "../subsystems/ConveyorSystem.hpp"
#include "../subsystems/MogoGrabSystem.hpp"

namespace devils
{
    struct TestAuto
    {
        static void runA(
            ChassisBase &chassis,
            OdomSource &odometry)
        {
            // PID Params
            PIDController::Options rotatePID = {0.7, 0.0, 50.0};

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
            pjRoutine.driveToTrajectory(-60, 60, 135, false)->run(); // 2 Rings in Corner
            pjRoutine.pause(1000)->run();                            // Wait to pickup rings
            pjRoutine.driveToTrajectory(-50, 50, 135, true)->run();  // Exit Corner
            pjRoutine.rotateTo(315)->run();                          // Turn Around
            pjRoutine.driveToTrajectory(-60, 60, 315, true)->run();  // Mogo in Corner
        }

        static void runB(
            ChassisBase &chassis,
            OdomSource &odometry)
        {
            AutoBuilder pjRoutine = AutoBuilder(chassis, odometry);
            pjRoutine.setPose(-50, -36, 180)->run();
            pjRoutine.pause(1000)->run();

            pjRoutine.driveToTrajectory(-10, -40, 135, true, 10)->run(); // Pickup Goal
            pjRoutine.driveToTrajectory(-24, -48, 220)->run();
        }
    };
}