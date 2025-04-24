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
            bool isBlue = false)
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
                0.3, // maxSpeed
                2.0, // goalDist
            };
            AutoRotateToStep::Options::defaultOptions = {
                rotatePID,
                0.15, // minSpeed
                0.4,  // maxSpeed
                0.05, // goalDist
            };
            TrajectoryConstraints slowConstraints = {24, 48};
            TrajectoryConstraints fastConstraints = {48, 92};

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

            // Mogo
            pjRoutine.setPose(-55, 12, 124)->run();
            pjRoutine.driveToTrajectory(-52, 0, 112.5, true, 0, 4)->run();
            mogoGrabber.setMogoGrabbed(true);

            // Wall Stake Ring
            pjRoutine.rotateTo(190)->run();
            conveyor.setPaused(true);
            pjRoutine.driveToTrajectory(-66, 2, 190, false, 0, 1, slowConstraints)->run();
            conveyor.setPaused(false);
            pjRoutine.pause(200)->run();
            pjRoutine.driveToTrajectory(-56, 0, 190, true, 0, 1, slowConstraints)->run();
            pjRoutine.rotateTo(90)->run();

            // Left Ring
            conveyor.setPaused(true);
            pjRoutine.driveToTrajectory(-52, 48, 70, false, 0, 6)->run();
            conveyor.setPaused(false);
            pjRoutine.pause(500)->run();

            // Edge Ring
            conveyor.setPaused(true);
            pjRoutine.driveToTrajectory(-18, 60, 0, false, 0, 8)->run();

            if (isBlue)
            {
                intake.setArmPosition(IntakeSystem::ArmPosition::INTAKE);
                conveyor.setArmLowered(false);
            }
            else
            {
                intake.setArmPosition(IntakeSystem::ArmPosition::BOTTOM_RING);
                conveyor.setArmLowered(true);
            }
            conveyor.setPaused(true);
            intake.setClawGrabbed(false);

            pjRoutine.rotateTo(0)->run();
            pjRoutine.driveToTrajectory(-8, 60, 0, false, 0, 3, slowConstraints)->run();
            intake.setClawGrabbed(true);
            pjRoutine.driveToTrajectory(-20, 60, 0, true, 0, 3, slowConstraints)->run();
            intake.setClawGrabbed(false);
            intake.setArmPosition(IntakeSystem::ArmPosition::INTAKE);
            conveyor.setArmLowered(false);
            pjRoutine.driveToTrajectory(-12, 60, 0, false, 0, 3, slowConstraints)->run();
            conveyor.setPaused(false);
            pjRoutine.pause(300)->run();

            // Right Ring
            pjRoutine.rotateTo(210)->run();
            pjRoutine.driveToTrajectory(-28, 36, 202, false, 0, 10)->run();

            // Corner
            intake.setArmPosition(IntakeSystem::ArmPosition::ALLIANCE_STAKE);
            pjRoutine.driveToTrajectory(-56, 56, 135, false, 0, 10)->run();
            conveyorStep->setTargetSpeed(0.6);
            conveyor.setPaused(true);
            pjRoutine.driveToTrajectory(-64, 64, 135, false, 0, 2)->run();
            conveyor.setPaused(false);
            pjRoutine.pause(4000)->run();
            conveyorStep->setTargetSpeed(1.0);

            // Drop Mogo
            pjRoutine.driveToTrajectory(-50, 50, 135, true, 0, 3)->run();
            intake.setArmPosition(IntakeSystem::ArmPosition::INTAKE);
            pjRoutine.rotateTo(315)->run();
            pjRoutine.driveToTrajectory(-60, 60, 315, true, 0, 3)->run();
            mogoGrabber.setMogoGrabbed(false);

            // End
            intake.setArmPosition(IntakeSystem::ArmPosition::NEUTRAL_STAKE);
            pjRoutine.driveToTrajectory(-18, 24, 315, false, 10, 10, fastConstraints)->run();

            // Stop
            intake.setArmPosition(IntakeSystem::ArmPosition::ALLIANCE_STAKE);
            pjRoutine.pause(2000)->run();
        }
    };
}