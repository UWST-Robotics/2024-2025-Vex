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
    struct BlazeSkillsAuto
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
                0.4,  // maxSpeed
                0.12, // minSpeed
                0.1,  // goalDist
                1.0,  // goalSpeed
            };

            // Speed Options
            TrajectoryConstraints slowConstraints = {24, 48};
            TrajectoryConstraints fastConstraints = {120, 120, 120};

            // Async Steps
            auto intakeStep = std::make_shared<AsyncIntakeStep>(intake);
            intakeStep->runAsync();

            auto conveyorStep = std::make_shared<AsyncConveyorStep>(conveyor, mogoGrabber);
            conveyorStep->runAsync();

            // Initial State
            AutoBuilder blazeRoutine = AutoBuilder(chassis, odometry);
            blazeRoutine.setPose(-64, 0, 0)->run();
            mogoGrabber.setMogoGrabbed(false);
            conveyor.setMogoDelayEnabled(false);
            conveyor.setRingSorting(RingType::BLUE);
            conveyor.setPaused(true);
            conveyor.setZoomEnabled(false);
            intake.setArmPosition(IntakeSystem::ArmPosition::INTAKE);

            // Red Wall Stake
            blazeRoutine.driveTrajectory(12.0, false, 0, 2, slowConstraints)->run();
            intakeOnce(conveyor, blazeRoutine);
            blazeRoutine.driveTrajectory(-12.0, true, 0, 2, slowConstraints)->run();
            mogoGrabber.setMogoGrabbed(true);
            blazeRoutine.pause(1000)->run();
            mogoGrabber.setMogoGrabbed(false);

            // SW Mogo
            blazeRoutine.driveTrajectory(17.0)->run();
            conveyor.setMogoDelayEnabled(true);
            blazeRoutine.rotateTo(Units::radToDeg(M_PI * 0.25))->run();
            blazeRoutine.driveTrajectory(30.0)->run();
            blazeRoutine.driveTrajectory(8, false, 0, 2, slowConstraints)->run();
            intakeOnce(conveyor, blazeRoutine);
            blazeRoutine.driveTrajectory(-4, true)->run();
            conveyor.setZoomEnabled(true);
            blazeRoutine.rotateTo(Units::radToDeg(M_PI * -0.5))->run();
            blazeRoutine.driveTrajectory(-23.0, true, 0, 2, slowConstraints)->run();
            mogoGrabber.setMogoGrabbed(true);
            intakeOnce(conveyor, blazeRoutine);

            blazeRoutine.rotateTo(Units::radToDeg(0))->run();
            blazeRoutine.driveTrajectory(24.0, false, 0, 2, slowConstraints)->run();
            intakeOnce(conveyor, blazeRoutine);

            blazeRoutine.driveTrajectory(-24.0, true)->run();
            blazeRoutine.rotateTo(Units::radToDeg(M_PI * 0.15))->run();
            blazeRoutine.driveTrajectory(24.0, false, 0, 2, slowConstraints)->run();
            intakeOnce(conveyor, blazeRoutine);
            blazeRoutine.driveTrajectory(-24.0, true)->run();

            blazeRoutine.rotateTo(Units::radToDeg(M_PI))->run();
            blazeRoutine.driveTrajectory(20)->run();
            blazeRoutine.driveTrajectory(7, false, 0, 2, slowConstraints)->run();
            intakeOnce(conveyor, blazeRoutine);
            blazeRoutine.pause(1000)->run();
            intake.setArmPosition(IntakeSystem::ArmPosition::ALLIANCE_STAKE);
            blazeRoutine.rotateTo(Units::radToDeg(M_PI * 0.75))->run();
            blazeRoutine.driveTrajectory(18)->run();
            intakeOnce(conveyor, blazeRoutine);
            blazeRoutine.driveTrajectory(-10.0, true)->run();
            blazeRoutine.rotateTo(Units::radToDeg(M_PI * -0.25))->run();
            intake.setArmPosition(IntakeSystem::ArmPosition::INTAKE);
            mogoGrabber.setMogoGrabbed(false);

            // SE Mogo
            blazeRoutine.driveTrajectory(-14.5, true)->run();

            blazeRoutine.driveTrajectory(22.0)->run();
            blazeRoutine.rotateTo(Units::radToDeg(0))->run();
            blazeRoutine.driveTrajectory(46.0)->run();
            blazeRoutine.rotateTo(Units::radToDeg(M_PI * 0.75))->run();
            blazeRoutine.driveTrajectory(-25.0, true)->run();
            blazeRoutine.driveTrajectory(-12.0, true, 0, 2, slowConstraints)->run();
            mogoGrabber.setMogoGrabbed(true);

            blazeRoutine.rotateTo(Units::radToDeg(M_PI * 0.5))->run();
            blazeRoutine.driveTrajectory(24.0, false, 0, 2, slowConstraints)->run();
            intakeOnce(conveyor, blazeRoutine);
            blazeRoutine.rotateTo(Units::radToDeg(0))->run();
            blazeRoutine.driveTrajectory(26.0, false, 0, 2, slowConstraints)->run();
            intakeOnce(conveyor, blazeRoutine);
            blazeRoutine.rotateTo(Units::radToDeg(M_PI * -0.5))->run();
            blazeRoutine.driveTrajectory(24.0, false, 0, 2, slowConstraints)->run();
            intakeOnce(conveyor, blazeRoutine);
            blazeRoutine.driveTrajectory(-24.0, true)->run();

            blazeRoutine.rotateTo(Units::radToDeg(M_PI * -0.75))->run();
            mogoGrabber.setMogoGrabbed(false);
            blazeRoutine.driveTrajectory(-16.0, true)->run();
            blazeRoutine.driveTrajectory(10.0)->run();
            blazeRoutine.driveTrajectory(-14.0, true)->run();
            blazeRoutine.driveTrajectory(20.0)->run();

            // Blue Wall Stake
            // blazeRoutine.rotateTo(Units::radToDeg(M_PI * -0.4))->run();

            // blazeRoutine.driveTrajectory(50)->run();
            // intakeOnce(conveyor, blazeRoutine);
            // blazeRoutine.rotateTo(Units::radToDeg(M_PI))->run();
            // blazeRoutine.driveTrajectory(16.0)->run();
            // blazeRoutine.driveTrajectory(-17, true, 0, 2, slowConstraints)->run();
            // mogoGrabber.setMogoGrabbed(true);
            // intakeOnce(conveyor, blazeRoutine);
        }

        static void intakeOnce(
            ConveyorSystem &conveyor,
            AutoBuilder &pjRoutine,
            uint32_t duration = 1000)
        {
            conveyor.setPaused(false);
            pjRoutine.pause(200)->run();

            auto pauseConveyorStep = std::make_shared<AsyncPauseConveyorStep>(conveyor, duration);
            pauseConveyorStep->runAsync();
        }
    };
}