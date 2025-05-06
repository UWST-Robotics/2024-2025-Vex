#pragma once

#include "devils/devils.h"
#include "../subsystems/IntakeSystem.hpp"
#include "../subsystems/ConveyorSystem.hpp"
#include "../subsystems/MogoGrabSystem.hpp"
#include "asyncIntakeStep.hpp"
#include "asyncConveyorStep.hpp"

namespace devils
{
    struct PJMatchAuto
    {
        static void southAuto(
            ChassisBase &chassis,
            OdomSource &odometry,
            IntakeSystem &intake,
            ConveyorSystem &conveyor,
            MogoGrabSystem &mogoGrabber,
            bool isBlue = false,
            bool endCenter = false)
        {
            // PID Params
            PIDController::Options rotatePID = {0.7, 0.0, 50.0};

            // Default Options
            AutoRotateToStep::Options::defaultOptions = {
                rotatePID,
                0.15, // minSpeed
                0.6,  // maxSpeed
                0.1,  // goalDist
            };
            TrajectoryConstraints slowConstraints = {24, 48};
            TrajectoryConstraints fastConstraints = {64, 92};

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

            // Mogo
            pjRoutine.setPose(-52, 12, 124)->run();
            pjRoutine.driveToTrajectory(-50, 0, 112.5, true, 0, 1)->run();
            mogoGrabber.setMogoGrabbed(true);
            pjRoutine.pause(100)->run();

            // Wall Stake Ring
            pjRoutine.rotateTo(200)->run();
            conveyor.setPaused(true);
            pjRoutine.driveToTrajectory(-62, -2, 200, false, 0, 1)->run();
            conveyor.setPaused(false);
            pjRoutine.pause(300)->run();
            pjRoutine.driveToTrajectory(-56, 0, 200, true, 0, 1)->run();
            pjRoutine.rotateTo(90)->run();

            // Left Ring
            conveyor.setPaused(true);
            intake.setArmPosition(IntakeSystem::ArmPosition::SECOND_RING);
            pjRoutine.driveToTrajectory(-49, 52, 70, false, 6, 6)->run();
            intake.setArmPosition(IntakeSystem::ArmPosition::INTAKE);
            conveyor.setPaused(false);

            // Edge Ring
            pjRoutine.driveToTrajectory(-18, 61, 0, false, 0, 8)->run();
            pjRoutine.rotateTo(0)->run();
            pjRoutine.pause(500)->run();

            if (isBlue)
                intake.setArmPosition(IntakeSystem::ArmPosition::INTAKE);
            else
                intake.setArmPosition(IntakeSystem::ArmPosition::BOTTOM_RING);
            conveyor.setPaused(true);

            pjRoutine.driveToTrajectory(-5, 61, 0, false, 0, 3)->run();
            intake.setClawGrabbed(true);
            pjRoutine.driveToTrajectory(-18, 61, 0, true, 0, 3)->run();
            intake.setClawGrabbed(false);
            intake.setArmPosition(IntakeSystem::ArmPosition::INTAKE);
            conveyor.setArmLowered(false);
            pjRoutine.driveToTrajectory(-12, 61, 0, false, 0, 3)->run();
            conveyor.setPaused(false);
            pjRoutine.pause(100)->run();

            // Loop Around
            pjRoutine.driveToTrajectory(-10, 48, 270, false, 6, 2, slowConstraints)->run();
            pjRoutine.driveToTrajectory(-30, 36, 170, false, 12, 2)->run();

            // Corner
            intake.setArmPosition(IntakeSystem::ArmPosition::ALLIANCE_STAKE);
            pjRoutine.driveToTrajectory(-53, 56, 135, false, 0, 10)->run();
            conveyorStep->setTargetSpeed(0.6);
            conveyor.setPaused(true);
            pjRoutine.driveToTrajectory(-63, 66, 135, false, 0, 2)->run();
            conveyor.setPaused(false);
            pjRoutine.pause(4000)->run();
            conveyorStep->setTargetSpeed(1.0);

            // Drop Mogo
            pjRoutine.driveToTrajectory(-50, 50, 135, true, 0, 3)->run();
            intake.setArmPosition(IntakeSystem::ArmPosition::INTAKE);
            pjRoutine.rotateTo(315)->run();
            mogoGrabber.setMogoGrabbed(false);
            pjRoutine.driveToTrajectory(-64, 64, 315, true, 0, 3)->run();
            pjRoutine.driveToTrajectory(-56, 56, 315, false, 0, 1)->run();
            pjRoutine.driveToTrajectory(-64, 64, 315, true, 0, 1)->run();
            pjRoutine.driveToTrajectory(-50, 50, 315, false, 0, 3)->run();

            // End
            if (endCenter)
            {
                conveyor.setPaused(true);
                pjRoutine.rotateTo(135)->run();
                pjRoutine.driveToTrajectory(-14, 15, 150, true, 0, 12)->run();

                // Clear rings out of the way
                pjRoutine.rotateTo(50)->run();
                pjRoutine.rotateTo(140)->run();
            }
            else
            {
                pjRoutine.rotateTo(180)->run();
                pjRoutine.driveToTrajectory(-10, 50, 180, true, 0, 10)->run();
            }

            // Stop
            pjRoutine.pause(999999)->run();
        }
    };
}