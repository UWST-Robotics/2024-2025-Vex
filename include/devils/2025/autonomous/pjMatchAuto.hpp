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
                0.6,  // maxSpeed
                0.2,  // goalDist
            };
            TrajectoryConstraints slowConstraints = {24, 48};
            TrajectoryConstraints fastConstraints = {64, 92};

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
            pjRoutine.setPose(-53, 12, 124)->run();
            pjRoutine.driveToTrajectory(-50, 0, 112.5, true, 0, 1)->run();
            mogoGrabber.setMogoGrabbed(true);

            // Wall Stake Ring
            pjRoutine.rotateTo(200)->run();
            conveyor.setPaused(true);
            pjRoutine.driveToTrajectory(-62, -2, 200, false, 0, 1, slowConstraints)->run();
            conveyor.setPaused(false);
            pjRoutine.pause(300)->run();
            pjRoutine.driveToTrajectory(-56, 0, 200, true, 0, 1, slowConstraints)->run();
            pjRoutine.rotateTo(90)->run();

            // Left Ring
            conveyor.setPaused(true);
            pjRoutine.driveToTrajectory(-49, 52, 70, false, 0, 6)->run();
            conveyor.setPaused(false);
            pjRoutine.pause(300)->run();

            // Edge Ring
            conveyor.setPaused(true);
            pjRoutine.driveToTrajectory(-18, 62, 0, false, 0, 8)->run();

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
            pjRoutine.driveToTrajectory(-5, 62, 0, false, 0, 3, slowConstraints)->run();
            intake.setClawGrabbed(true);
            pjRoutine.driveToTrajectory(-18, 62, 0, true, 0, 3, slowConstraints)->run();
            intake.setClawGrabbed(false);
            intake.setArmPosition(IntakeSystem::ArmPosition::INTAKE);
            conveyor.setArmLowered(false);
            pjRoutine.driveToTrajectory(-12, 62, 0, false, 0, 3, slowConstraints)->run();
            conveyor.setPaused(false);
            pjRoutine.pause(300)->run();

            // Right Ring
            pjRoutine.rotateTo(210)->run();
            conveyor.setPaused(true);
            pjRoutine.driveToTrajectory(-28, 36, 202, false, 0, 10)->run();
            conveyor.setPaused(false);
            pjRoutine.pause(300)->run();

            // Corner
            intake.setArmPosition(IntakeSystem::ArmPosition::ALLIANCE_STAKE);
            pjRoutine.driveToTrajectory(-56, 56, 135, false, 0, 10)->run();
            conveyorStep->setTargetSpeed(0.6);
            conveyor.setPaused(true);
            pjRoutine.driveToTrajectory(-66, 66, 135, false, 0, 2)->run();
            conveyor.setPaused(false);
            pjRoutine.pause(4000)->run();
            conveyorStep->setTargetSpeed(1.0);

            // Drop Mogo
            pjRoutine.driveToTrajectory(-50, 50, 135, true, 0, 3)->run();
            intake.setArmPosition(IntakeSystem::ArmPosition::INTAKE);
            pjRoutine.rotateTo(315)->run();
            pjRoutine.driveToTrajectory(-60, 60, 315, true, 0, 3)->run();
            mogoGrabber.setMogoGrabbed(false);
            pjRoutine.driveToTrajectory(-50, 50, 315, false, 0, 3)->run();

            // End
            if (endCenter)
            {
                pjRoutine.rotateTo(135)->run();
                pjRoutine.driveToTrajectory(-10, 10, 135, true, 0, 10)->run();
            }
            else
            {
                pjRoutine.rotateTo(180)->run();
                pjRoutine.driveToTrajectory(-10, 50, 180, true, 0, 10)->run();
            }

            // Stop
            pjRoutine.pause(2000)->run();
        }
    };
}