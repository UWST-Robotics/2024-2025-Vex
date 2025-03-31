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
            PIDController::Options drivePID = {0.15, 0.0, 10};
            PIDController::Options rotatePID = {0.7, 0.0, 50.0};
            PIDController::Options drivingRotatePID = {1.5, 0.0, 100.0};

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
                0.15, // minSpeed
                0.4,  // maxSpeed
                0.05, // goalDist
                0.5,  // goalSpeed
            };

            // Initialize
            AutoBuilder pjRoutine = AutoBuilder(chassis, odometry);
            auto step = pjRoutine.setPose(-66, -48, 0);
        }
    };
}