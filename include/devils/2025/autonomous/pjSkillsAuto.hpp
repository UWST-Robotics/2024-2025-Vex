#pragma once

#include "devils/devils.h"
#include "../subsystems/IntakeSystem.hpp"
#include "../subsystems/ConveyorSystem.hpp"
#include "../subsystems/MogoGrabSystem.hpp"

namespace devils
{
    struct PJSkillsAuto
    {
        static void runSkills(
            ChassisBase &chassis,
            OdomSource &odometry,
            IntakeSystem &intake,
            ConveyorSystem &conveyor,
            MogoGrabSystem &mogoGrabber)
        {
            // PID Params
            PIDParams drivePID = {0.15, 0.0, 10};
            PIDParams rotatePID = {0.7, 0.0, 50.0};
            PIDParams drivingRotatePID = {1.5, 0.0, 100.0};

            // Default Options
            AutoDriveToStep::Options::defaultOptions = {
                drivePID,
                drivingRotatePID,
                0.5, // maxSpeed
                2.0, // goalDist
                2.0, // goalSpeed
            };
            AutoRotateToStep::Options::defaultOptions = {
                rotatePID,
                0.4,  // maxSpeed
                0.15, // minSpeed
                0.05, // goalDist
                0.5,  // goalSpeed
            };

            // Speed Options
            AutoDriveToStep::Options mogoGrabSpeed = {
                drivePID,
                drivingRotatePID,
                0.2, // maxSpeed
                1.0, // goalDist
                2.0  // goalSpeed
            };
            AutoDriveToStep::Options intakeSpeed = {
                drivePID,
                drivingRotatePID,
                0.25, // maxSpeed
                1.0,  // goalDist
                2.0   // goalSpeed
            };
            AutoDriveToStep::Options cornerSpeed = {
                drivePID,
                drivingRotatePID,
                0.2, // maxSpeed
                2.0, // goalDist
                2.0  // goalSpeed
            };

            // Initialize
            AutoBuilder pjRoutine = AutoBuilder(chassis, odometry);
            pjRoutine.setPose(-66, -48, 0)->run();

            // TODO: Program the rest of the autonomous routine
            pjRoutine.driveTo(0, 0, 0)->run();
        }
    };
}