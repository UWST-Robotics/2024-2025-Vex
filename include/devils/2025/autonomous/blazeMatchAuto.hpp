#pragma once

#include "devils/devils.h"
#include "../subsystems/IntakeSystem.hpp"
#include "../subsystems/ConveyorSystem.hpp"
#include "../subsystems/MogoGrabSystem.hpp"
#include "../subsystems/GoalRushSystem.hpp"
#include "asyncIntakeStep.hpp"
#include "asyncConveyorStep.hpp"
#include "asyncPauseConveyorStep.hpp"

namespace devils
{
    struct BlazeMatchAuto
    {
        static void runMatch(
            ChassisBase &chassis,
            OdomSource &odometry,
            IntakeSystem &intake,
            ConveyorSystem &conveyor,
            MogoGrabSystem &mogoGrabber,
            GoalRushSystem &goalRushSystem,
            bool isBlue = false)
        {
            // PID Params
            PIDController::Options rotatePID = {0.6, 0.0, 50.0};

            // Default Options
            AutoRotateToStep::Options::defaultOptions = {
                rotatePID,
                0.2,  // minSpeed
                0.6,  // maxSpeed
                0.03, // goalDist
                0.2   // goalSpeed
            };
            TrajectoryConstraints slowConstraints = {24, 48};
            TrajectoryConstraints fastConstraints = {200, 200, 200};

            // Async Steps
            auto intakeStep = std::make_shared<AsyncIntakeStep>(intake);
            intakeStep->runAsync();

            auto conveyorStep = std::make_shared<AsyncConveyorStep>(conveyor, mogoGrabber);
            conveyorStep->runAsync();

            // Initialize
            AutoBuilder pjRoutine = AutoBuilder(chassis, odometry);
            pjRoutine.useTransformer(isBlue ? std::make_unique<MirrorTransformX>() : nullptr);
            intake.setArmPosition(IntakeSystem::INTAKE);
            mogoGrabber.setMogoGrabbed(false);
            conveyor.setRingSorting(isBlue ? RingType::RED : RingType::BLUE);
            conveyor.setPaused(true);
            conveyor.setZoomEnabled(true);
            pjRoutine.setPose(-48, -61, 189)->run();

            // Mogo Rush
            auto mogoRushStep = pjRoutine.driveToTrajectory(isBlue ? -17 : -14, isBlue ? -42 : -52, isBlue ? 220 : 189, true, 0, 2, fastConstraints);
            mogoRushStep->runAsync();

            // Release String Mech
            intake.setArmPosition(IntakeSystem::INTAKE);
            pjRoutine.pause(100)->run(); // <-- Wait for string mech to retract
            intake.setArmPosition(IntakeSystem::BOTTOM_RING);

            // Mogo Rush Mech
            goalRushSystem.setExtended(true);
            mogoRushStep->joinAsync();
            pjRoutine.pause(100)->run(); // <-- Wait for claw to get into position
            goalRushSystem.setClamped(true);
            goalRushSystem.setExtended(false);
            pjRoutine.pause(200)->run(); // <-- Wait for pneumatic to actuate

            // Pull
            pjRoutine.driveToTrajectory(-40, isBlue ? -58 : -62, 189, false, 0, 2, fastConstraints)->run();

            // Fallback to Center Goal
            bool hasMogo = goalRushSystem.hasMogo();
            if (!hasMogo)
            {
                // Drive to Center Goal
                pjRoutine.rotateTo(270)->run();
                pjRoutine.driveToTrajectory(-24, -14, isBlue ? 235 : 215, true, 0, 8)->run();
                goalRushSystem.setExtended(true);
                goalRushSystem.setClamped(false);
                pjRoutine.driveToTrajectory(isBlue ? -11 : -12, isBlue ? -6 : -8, isBlue ? 230 : 210, true, 0, 1)->run();

                // Pull
                goalRushSystem.setClamped(true);
                goalRushSystem.setExtended(false);
                pjRoutine.driveToTrajectory(-36, -20, 190, false, 0, 2)->run();

                // Release
                goalRushSystem.setClamped(false);
                goalRushSystem.setExtended(true);
                pjRoutine.driveToTrajectory(-44, -21, 180, false, 0, 2, slowConstraints)->run();
                goalRushSystem.setExtended(false);

                // Grab Mogo
                pjRoutine.driveToTrajectory(-16, isBlue ? -14 : -2, isBlue ? 210 : 230, true, 0, 2, slowConstraints)->run();
                mogoGrabber.setMogoGrabbed(true);

                // Ring 1
                conveyor.setPaused(true);
                pjRoutine.driveToTrajectory(-20, -30, 210, false, 12, 1, fastConstraints)->run();
                pjRoutine.driveToTrajectory(-40, -46, 210, false, 12, 12, slowConstraints)->run();
                pjRoutine.rotateTo(180)->run();
                pjRoutine.pause(1000)->run();
                intake.setArmPosition(IntakeSystem::BOTTOM_RING);
                pjRoutine.driveToTrajectory(-58, -46, 180, false, 0, 2, slowConstraints)->run();
                intake.setClawGrabbed(true);
                pjRoutine.pause(200)->run(); // <-- Wait for clamp actuation
                pjRoutine.driveToTrajectory(-45, -46, 180, true, 0, 2)->run();
                // pjRoutine.setPose(-48, -46, 0)->run(); // <-- Set pose to account for drift
            }
            else
            {
                // Release
                goalRushSystem.setExtended(true);
                goalRushSystem.setClamped(false);
                pjRoutine.driveToTrajectory(-50, -62, 180, false, 0, 2, slowConstraints)->run();

                // Grab Mogo
                goalRushSystem.setExtended(false);
                pjRoutine.pause(3000)->run(); // <-- Wait for mogo to stop oscillating
                pjRoutine.driveToTrajectory(-12, isBlue ? -54 : -50, 189, true, 0, 4, slowConstraints)->run();
                mogoGrabber.setMogoGrabbed(true);

                // Ring 1
                conveyor.setPaused(true);
                pjRoutine.driveToTrajectory(-48, -46, 160, false, 12, 6)->run();
                pjRoutine.pause(500)->run();
                intake.setArmPosition(IntakeSystem::BOTTOM_RING);
                pjRoutine.driveToTrajectory(-56, -38, 160, false, 0, 2)->run();
                intake.setClawGrabbed(true);
                pjRoutine.pause(200)->run(); // <-- Wait for clamp actuation
                pjRoutine.driveToTrajectory(-48, -46, 160, true, 0, 1)->run();
            }

            // Corner
            pjRoutine.rotateTo(220)->run();
            intake.setArmPosition(IntakeSystem::ALLIANCE_STAKE);
            pjRoutine.pause(100)->run(); // <-- Wait for rotate to finish
            pjRoutine.driveToTrajectory(hasMogo ? -69 : -63, isBlue ? -59 : -62, 220, false, 0, 4, slowConstraints)->run();
            if (isBlue)
                pjRoutine.driveTrajectory(6)->run();
            conveyorStep->setTargetSpeed(0.7);
            conveyor.setPaused(false);
            pjRoutine.pause(4000)->run(); // <-- Wait for corner to clear

            // Wall Stake
            pjRoutine.driveToTrajectory(-24, -24, 190, true, 0, 12)->run();
            pjRoutine.rotateTo(135)->run();
            intake.setArmPosition(IntakeSystem::ALLIANCE_STAKE);
            pjRoutine.driveToTrajectory(-50, hasMogo ? 0 : -1.5, 180, false, 0, 12)->run();
            pjRoutine.rotateTo(180)->run();
            pjRoutine.driveToTrajectory(-67, hasMogo ? 0 : -1.5, 180, false, 0, 6)->run();
            intake.setClawGrabbed(false);
            intake.setArmPosition(IntakeSystem::NEUTRAL_STAKE);
            pjRoutine.driveToTrajectory(-60, hasMogo ? 0 : -1.5, 180, true, 0, 1)->run();
            intake.setArmPosition(IntakeSystem::ALLIANCE_STAKE);
            pjRoutine.pause(500)->run(); // <-- Wait for arm to move
            intake.setArmPosition(IntakeSystem::NEUTRAL_STAKE);

            // Finish Corner
            pjRoutine.driveToTrajectory(-42, 0, 180, true, 0, 2)->run();
            intake.setArmPosition(IntakeSystem::INTAKE);
            pjRoutine.driveToTrajectory(-50, 50, 90, false, 0, 8, slowConstraints)->run();

            // Stop Async Steps
            intakeStep->stop();
            conveyorStep->stop();
        }
    };
}