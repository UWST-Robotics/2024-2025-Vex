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
            PIDController::Options rotatePID = {0.7, 0.0, 50.0};

            // Default Options
            AutoRotateToStep::Options::defaultOptions = {
                rotatePID,
                0.14, // minSpeed
                0.4,  // maxSpeed
                0.05, // goalDist
                0.15  // goalSpeed

            };
            TrajectoryConstraints slowConstraints = {24, 48};
            TrajectoryConstraints fastConstraints = {120, 120, 120};

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

            // Ring 1
            pjRoutine.setPose(-66, -48, 0)->run();
            pjRoutine.driveToTrajectory(-48, -48, 0, false, 0, 2)->run();
            intakeOnce(conveyor, pjRoutine);
            pjRoutine.pause(200)->run();

            // Mogo
            pjRoutine.rotateTo(180)->run();
            pjRoutine.driveToTrajectory(-20, -48, 180, true, 0, 2)->run();
            mogoGrabber.setMogoGrabbed(true);
            intakeOnce(conveyor, pjRoutine);

            // Ring 2
            pjRoutine.rotateTo(0)->run();
            pjRoutine.driveToTrajectory(4, -48, 0, false, 0, 2)->run();
            intakeOnce(conveyor, pjRoutine);

            // Ring 3
            pjRoutine.driveToTrajectory(24, -48, 0, false, 0, 2)->run();
            intakeOnce(conveyor, pjRoutine);
            pjRoutine.pause(200)->run();

            // Ring 4
            pjRoutine.rotateTo(180)->run();
            pjRoutine.driveToTrajectory(12, -48, 180, false, 12, 12)->run();
            pjRoutine.driveToTrajectory(-24, -24, 170, false, 12, 8)->run();
            pjRoutine.driveToTrajectory(-28, -20, 135, false, 0, 1)->run();
            intakeOnce(conveyor, pjRoutine);

            // Corner
            pjRoutine.driveToTrajectory(-52, -51, 225, false, 0, 12, slowConstraints)->run();
            pjRoutine.driveToTrajectory(-67, -66, 225, false, 0, 12, fastConstraints)->run();
            intakeOnce(conveyor, pjRoutine);

            // Drop Mogo
            pjRoutine.driveToTrajectory(-50, -50, 225, true, 0, 2)->run();
            pjRoutine.rotateTo(45)->run();
            pjRoutine.driveToTrajectory(-60, -60, 225, true, 0, 2)->run();
            mogoGrabber.setMogoGrabbed(false);
            pjRoutine.driveToTrajectory(-50, -50, 225, false, 0, 2)->run();

            // Wall Stake
            pjRoutine.rotateTo(0)->run();
            intake.setArmPosition(IntakeSystem::ArmPosition::BOTTOM_RING);
            pjRoutine.driveToTrajectory(-4, -60, 0, false, 12, 18)->run();
            pjRoutine.driveToTrajectory(4, -60, 0, false, 0, 2)->run();

            intake.setClawGrabbed(true);
            pjRoutine.pause(200)->run();

            intake.setArmPosition(IntakeSystem::ArmPosition::NEUTRAL_STAKE);
            pjRoutine.driveToTrajectory(-7, -59, 0, true, 0, 2)->run();
            pjRoutine.rotateTo(270)->run();
            pjRoutine.driveToTrajectory(-2, -40, 270, true, 0, 3)->run();
            pjRoutine.driveToTrajectory(-2, -70, 270, false, 0, 1, fastConstraints)->run();

            intake.setClawGrabbed(false);
            intake.setArmPosition(IntakeSystem::ArmPosition::ABOVE_NEUTRAL_STAKE);
            pjRoutine.pause(200)->run();

            pjRoutine.driveToTrajectory(-2, -48, 270, true, 0, 3)->run();
            intake.setArmPosition(IntakeSystem::ArmPosition::INTAKE);

            // Mogo
            pjRoutine.rotateTo(225)->run();
            pjRoutine.driveToTrajectory(30, -18, 225, true, 0, 2)->run();
            mogoGrabber.setMogoGrabbed(true);
            pjRoutine.driveToTrajectory(22, -26, 225, false, 0, 2)->run();
            pjRoutine.rotateTo(135)->run();

            // Center Rings
            pjRoutine.driveToTrajectory(0, 0, 135, false, 0, 1, slowConstraints)->run();
            intakeOnce(conveyor, pjRoutine);
            pjRoutine.driveToTrajectory(-12, 12, 135, false, 0, 1, slowConstraints)->run();
            intakeOnce(conveyor, pjRoutine);
            pjRoutine.driveToTrajectory(3, -3, 135, true, 0, 1, slowConstraints)->run();
            pjRoutine.driveToTrajectory(-10, 0, 210, false, 0, 1, slowConstraints)->run();
            intakeOnce(conveyor, pjRoutine);
            pjRoutine.driveToTrajectory(3, -3, 135, true, 0, 1, slowConstraints)->run();
            pjRoutine.driveToTrajectory(6, 10, 60, false, 0, 1, slowConstraints)->run();
            intakeOnce(conveyor, pjRoutine);
            pjRoutine.driveToTrajectory(3, -3, 135, true, 0, 1, slowConstraints)->run();

            // Mogo
            mogoGrabber.setMogoGrabbed(false);
            pjRoutine.driveToTrajectory(26, -40, 115, true, 0, 12)->run();
            pjRoutine.driveToTrajectory(24, -24, 135, false, 0, 2)->run();
            pjRoutine.rotateTo(235)->run();
            pjRoutine.driveToTrajectory(52, 4, 225, true, 0, 2)->run();
            mogoGrabber.setMogoGrabbed(true);
            pjRoutine.pause(200)->run();
            pjRoutine.driveToTrajectory(48, 0, 225, false, 0, 1)->run();

            // Ring 1
            pjRoutine.rotateTo(270)->run();
            pjRoutine.driveToTrajectory(48, -24, 270, false, 0, 2)->run();
            intakeOnce(conveyor, pjRoutine);

            // Clear Blue Ring
            pjRoutine.driveToTrajectory(48, -40, 270, false, 0, 2)->run();
            intakeOnce(conveyor, pjRoutine);
            pjRoutine.pause(200)->run();

            // Ring 2
            pjRoutine.driveToTrajectory(48, -56, 270, false, 0, 2)->run();
            intakeOnce(conveyor, pjRoutine);
            pjRoutine.driveToTrajectory(48, -48, 270, true, 0, 1)->run();

            // Corner
            pjRoutine.rotateTo(315)->run();
            pjRoutine.driveToTrajectory(52, -54, 315, false, 0, 2, slowConstraints)->run();
            intakeOnce(conveyor, pjRoutine);
            intake.setArmPosition(IntakeSystem::ArmPosition::NEUTRAL_STAKE);
            pjRoutine.pause(500)->run();
            pjRoutine.driveToTrajectory(62, -64, 315, false, 0, 2, fastConstraints)->run();
            intakeOnce(conveyor, pjRoutine);

            // Drop Mogo
            pjRoutine.driveToTrajectory(50, -50, 315, true, 0, 2)->run();
            pjRoutine.rotateTo(135)->run();
            mogoGrabber.setMogoGrabbed(false);
            pjRoutine.driveToTrajectory(60, -60, 135, true, 0, 2)->run();
            pjRoutine.driveToTrajectory(50, -50, 135, false, 0, 2)->run();
            pjRoutine.driveToTrajectory(60, -60, 135, true, 0, 2)->run();

            // Climb
            intake.setArmPosition(IntakeSystem::ArmPosition::NEUTRAL_STAKE);
            pjRoutine.driveToTrajectory(16, -16, 135, false, 0, 2)->run();

            /*
            pjRoutine.setPose(32, -12, 180)->run();
            mogoGrabber.setMogoGrabbed(true);
            pjRoutine.pause(500)->run();

            // Center 1-2
            pjRoutine.driveToTrajectory(8, -6, 180, false, 0, 2)->run();
            intakeOnce(conveyor, pjRoutine);
            pjRoutine.driveToTrajectory(0, -6, 180, false, 0, 2, fastConstraints)->run();
            pjRoutine.pause(500)->run();
            pjRoutine.driveToTrajectory(-9, -6, 180, false, 0, 2)->run();
            intakeOnce(conveyor, pjRoutine);

            // Center 3-4
            pjRoutine.driveToTrajectory(13, -7, 150, true, 0, 2)->run();
            pjRoutine.driveToTrajectory(5, 5, 150, false, 0, 4)->run();
            intakeOnce(conveyor, pjRoutine, 2000);
            pjRoutine.driveToTrajectory(-10, 5, 180, false, 0, 4)->run();
            pjRoutine.pause(500)->run();
            */

            pjRoutine.pause(99999999)->run();
        }

        static void intakeOnce(
            ConveyorSystem &conveyor,
            AutoBuilder &pjRoutine,
            uint32_t duration = 1000)
        {
            conveyor.setPaused(false);
            pjRoutine.pause(100)->run();

            // TODO: Fix Memory Leak Here
            auto pauseConveyorStep = new AsyncPauseConveyorStep(conveyor, duration);
            pauseConveyorStep->runAsync();
        }
    };
}