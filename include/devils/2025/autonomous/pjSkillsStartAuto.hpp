#pragma once

#include "devils/devils.h"
#include "../subsystems/IntakeSystem.hpp"
#include "../subsystems/ConveyorSystem.hpp"
#include "../subsystems/MogoGrabSystem.hpp"
#include "asyncIntakeStep.hpp"
#include "asyncConveyorStep.hpp"
#include "asyncPauseConveyorStep.hpp"

namespace devils
{
    struct PJSkillsStartAuto
    {
        static void runStart(
            ChassisBase &chassis,
            OdomSource &odometry,
            IntakeSystem &intake,
            ConveyorSystem &conveyor,
            MogoGrabSystem &mogoGrabber)
        {
            // PID Params
            PIDController::Options rotatePID = {0.7, 0.0, 50.0};

            // Default Options
            AutoRotateToStep::Options::defaultOptions = {
                rotatePID,
                0.14, // minSpeed
                1,    // maxSpeed
                0.05  // goalDist

            };
            TrajectoryConstraints fastConstraints = {200, 200, 200};
            TrajectoryConstraints defaultConstraints = {200, 200, 80};

            AutoRamseteStep::Options sensitiveRamsete = {
                0.1,  // minSpeed
                1.0,  // maxSpeed
                0.02, // proportionGain
                0.6   // dampingCoefficient
            };

            // Async Steps
            auto intakeStep = std::make_unique<AsyncIntakeStep>(intake);
            intakeStep->runAsync();

            auto conveyorStep = std::make_unique<AsyncConveyorStep>(conveyor, mogoGrabber);
            conveyorStep->runAsync();

            // Initialize
            AutoBuilder pjRoutine = AutoBuilder(chassis, odometry);
            intake.setArmPosition(IntakeSystem::INTAKE);
            mogoGrabber.setMogoGrabbed(false);
            conveyor.setRingSorting(RingType::BLUE);
            conveyor.setPaused(true);
            conveyor.setMogoDelayEnabled(false);

            // Ring 1
            pjRoutine.setPose(-66, 0, 0)->run();
            pjRoutine.driveToTrajectory(-50, 0, 0, false, 0, 2, fastConstraints)->run();
            conveyor.setPaused(false);
            pjRoutine.pause(200)->run();

            // Mogo
            pjRoutine.driveToTrajectory(-66.2, -0.5, 0, true, 0, 2, fastConstraints, sensitiveRamsete)->run();
            chassis.stop();
            pjRoutine.pause(400)->run();
            mogoGrabber.setMogoGrabbed(true);
            pjRoutine.pause(700)->run();
            mogoGrabber.setMogoGrabbed(false);

            // Ring 2
            pjRoutine.driveToTrajectory(-60, 0, 0, false, 0, 2, fastConstraints)->run();
            pjRoutine.rotateTo(225)->run();
            pjRoutine.driveToTrajectory(-32, 40, 225, true, 18, 4, defaultConstraints)->run();
            pjRoutine.driveToTrajectory(-24, 48, 225, true, 0, 4)->run();
            mogoGrabber.setMogoGrabbed(true);

            // Ring 3
            pjRoutine.driveToTrajectory(-54, 48, 135, false, 12, 4)->run();
            pjRoutine.pause(200)->run();

            // Ring 4
            pjRoutine.driveToTrajectory(-72, 66, 135, false, 0, 4, fastConstraints)->run();
            pjRoutine.pause(500)->run();

            // Revert to Default
            conveyor.setMogoDelayEnabled(true);
            conveyor.setRingSorting(RingType::NONE);

            // Stop Async Steps
            intakeStep->stop();
            conveyorStep->stop();
        }
    };
}