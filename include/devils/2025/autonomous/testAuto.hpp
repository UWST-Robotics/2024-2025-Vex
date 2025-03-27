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
            PIDParams drivePID = {0.15, 0.0, 10};
            PIDParams rotatePID = {0.7, 0.0, 50.0};
            PIDParams drivingRotatePID = {1.5, 0.0, 100.0};

            // Default Options
            AutoDriveToStep::Options::defaultOptions = {
                drivePID,
                drivingRotatePID,
                0.0, // minSpeed
                0.5, // maxSpeed
                2.0, // goalDist
                2.0, // goalSpeed
            };
            AutoRotateToStep::Options::defaultOptions = {
                rotatePID,
                0.1,  // minSpeed
                0.5,  // maxSpeed
                0.05, // goalDist
                0.5,  // goalSpeed
            };

            // Create Test Path
            SplinePath testPath = SplinePath::makeArc(Pose(-48, -36, 0), Pose(-10, -44, Units::degToRad(-30)));
            VBPath::sync("testPath", testPath);

            // Initialize
            AutoBuilder pjRoutine = AutoBuilder(chassis, odometry);
            pjRoutine.setPose(-48, -36, 0)->run();
            pjRoutine.pause(1000)->run();
            pjRoutine.driveTo(-10, -44, -30)->run();
            chassis.stop();
        }
    };
}