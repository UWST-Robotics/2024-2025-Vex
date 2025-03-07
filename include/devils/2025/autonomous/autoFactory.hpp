#pragma once

#include "devils/devils.h"
#include "autoIntakeMoveArmStep.hpp"
#include "autoIntakeSetArmPositionStep.hpp"
#include "autoIntakeClawStep.hpp"
#include "autoMogoStep.hpp"
#include "autoConveyorStep.hpp"
#include "autoMogoBranchStep.hpp"
#include "autoConveyorPickupStep.hpp"
#include "autoSortStep.hpp"

namespace devils
{
    class AutoFactory
    {
    public:
        static AutoStepList *createPJSkillsAuto(
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

            // Initial State
            AutoBuilder pjRoutine = AutoBuilder(chassis, odometry);
            pjRoutine.setPose(-66, -48, 0);
            pjRoutine.addAsyncStep(new AutoIntakeMoveArmStep(intake));
            pjRoutine.addAsyncStep(new AutoConveyorStep(conveyor, intake, mogoGrabber));
            pjRoutine.addStep(new AutoIntakeSetArmPositionStep(intake, IntakeSystem::ArmPosition::INTAKE));
            pjRoutine.addStep(new AutoIntakeClawStep(intake, false));
            pjRoutine.addStep(new AutoSortStep(conveyor, RingType::BLUE));
            pjRoutine.pause(500); // Wait for intake to extend

            // NE Mogo
            pjRoutine.drive(18.0);
            pjRoutine.addStep(new AutoConveyorPickupStep(conveyor, true, 1000));
            pjRoutine.rotateTo(M_PI);
            pjRoutine.driveRelative(-16.0);
            pjRoutine.rotateTo(M_PI);
            pjRoutine.driveRelative(-12.0, 1500, mogoGrabSpeed);
            pjRoutine.addStep(new AutoGrabMogoStep(mogoGrabber, true));
            pjRoutine.driveRelative(8);

            pjRoutine.rotateTo(M_PI * -0.12);
            pjRoutine.drive(18.0);
            pjRoutine.drive(12.0, 1000, intakeSpeed);
            pjRoutine.addStep(new AutoConveyorPickupStep(conveyor, true, 1000));
            pjRoutine.drive(-30.0);

            pjRoutine.rotateTo(M_PI * 0.5);
            pjRoutine.drive(19.0);
            pjRoutine.driveRelative(8.0, 2000, intakeSpeed);
            pjRoutine.addStep(new AutoConveyorPickupStep(conveyor, true, 1000));
            pjRoutine.driveRelative(-8);
            pjRoutine.rotateTo(M_PI * -0.75);
            pjRoutine.addStep(new AutoIntakeSetArmPositionStep(intake, IntakeSystem::ArmPosition::NEUTRAL_STAKE));
            pjRoutine.drive(48.0);

            pjRoutine.addStep(new AutoConveyorPickupStep(conveyor, true, 1000));

            pjRoutine.drive(-12.0);
            pjRoutine.rotateTo(M_PI * 0.25);
            pjRoutine.addStep(new AutoIntakeSetArmPositionStep(intake, IntakeSystem::ArmPosition::INTAKE));
            pjRoutine.addStep(new AutoGrabMogoStep(mogoGrabber, false));
            pjRoutine.driveRelative(8.0);
            pjRoutine.drive(-20.0, 1000);

            // Center Mogo
            pjRoutine.drive(15.0);
            pjRoutine.addStep(new AutoConveyorPickupStep(conveyor, true, 1000));
            pjRoutine.rotateTo(0);
            pjRoutine.drive(54.0);
            pjRoutine.addStep(new AutoConveyorPickupStep(conveyor, true, 1000));
            pjRoutine.rotateTo(M_PI * -0.75);
            pjRoutine.drive(-32.0);
            pjRoutine.drive(-12.0, 1500, mogoGrabSpeed);
            pjRoutine.addStep(new AutoGrabMogoStep(mogoGrabber, true));
            pjRoutine.driveRelative(8);

            pjRoutine.rotateTo(M_PI * 0.75);
            pjRoutine.drive(32, 2000, intakeSpeed);
            pjRoutine.addStep(new AutoConveyorPickupStep(conveyor, true, 500));
            pjRoutine.drive(-6);
            pjRoutine.rotate(M_PI * 0.15);
            pjRoutine.rotate(M_PI * -0.3);
            pjRoutine.drive(12, 1000, intakeSpeed);
            pjRoutine.addStep(new AutoConveyorPickupStep(conveyor, true, 500));
            pjRoutine.drive(-12);
            pjRoutine.rotate(M_PI * 0.15);
            pjRoutine.drive(12, 1000, intakeSpeed);
            pjRoutine.addStep(new AutoConveyorPickupStep(conveyor, true, 500));
            pjRoutine.drive(-12);

            pjRoutine.rotateTo(M_PI * -0.25);

            pjRoutine.addStep(new AutoGrabMogoStep(mogoGrabber, false));
            pjRoutine.drive(26.0);

            // NW Mogo
            pjRoutine.rotateTo(M_PI * -0.75);
            pjRoutine.drive(-28.0);
            pjRoutine.drive(-12.0, 1500, mogoGrabSpeed);
            pjRoutine.addStep(new AutoGrabMogoStep(mogoGrabber, true));
            pjRoutine.driveRelative(8);

            pjRoutine.rotateTo(M_PI * -0.5);
            pjRoutine.drive(20.0);
            pjRoutine.driveRelative(8.0, 2000, intakeSpeed);
            pjRoutine.addStep(new AutoConveyorPickupStep(conveyor, true, 1000));
            pjRoutine.driveRelative(-4.0);
            pjRoutine.rotateTo(M_PI * -0.75);
            pjRoutine.drive(30.0);
            pjRoutine.driveRelative(8.0, 2000, intakeSpeed);
            pjRoutine.addStep(new AutoConveyorPickupStep(conveyor, true, 1000));
            pjRoutine.driveRelative(-4.0);
            pjRoutine.rotateTo(0);
            pjRoutine.drive(20.0);
            pjRoutine.driveRelative(8.0, 2000, intakeSpeed);
            pjRoutine.addStep(new AutoConveyorPickupStep(conveyor, true, 1000));
            pjRoutine.driveRelative(-4.0);

            pjRoutine.rotateTo(M_PI * 0.75);
            pjRoutine.addStep(new AutoGrabMogoStep(mogoGrabber, false));
            pjRoutine.drive(6.0);
            pjRoutine.drive(-22.0);

            // Climb
            pjRoutine.drive(28.0);

            return pjRoutine.build();
        }

        static AutoStepList *createPJMatchAuto(
            ChassisBase &chassis,
            OdomSource &odometry,
            IntakeSystem &intake,
            ConveyorSystem &conveyor,
            MogoGrabSystem &mogoGrabber,
            bool isBlue = false)
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
                0.5,  // maxSpeed
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

            AutoBuilder pjRoutine = AutoBuilder(chassis, odometry);

            // Initial State
            pjRoutine.useTransformer(isBlue ? new MirrorTransform() : nullptr);
            pjRoutine.setPose(-42, -36, M_PI);
            pjRoutine.addAsyncStep(new AutoIntakeMoveArmStep(intake));
            AutoAsyncStep *conveyorStep = pjRoutine.addAsyncStep(new AutoConveyorStep(conveyor, intake, mogoGrabber));
            pjRoutine.addStep(new AutoIntakeSetArmPositionStep(intake, IntakeSystem::ArmPosition::INTAKE));
            pjRoutine.addStep(new AutoIntakeClawStep(intake, false));
            pjRoutine.addStep(new AutoSortStep(conveyor, isBlue ? RingType::RED : RingType::BLUE));

            // Step 1
            pjRoutine.addStep(new AutoGrabMogoStep(mogoGrabber, false));
            pjRoutine.drive(-28.0);
            pjRoutine.rotateTo(M_PI * 0.85);
            pjRoutine.drive(-19.5, 2000, mogoGrabSpeed);
            pjRoutine.addStep(new AutoGrabMogoStep(mogoGrabber, true));
            pjRoutine.addStopAsyncStep(conveyorStep);
            pjRoutine.driveRelative(34.0, 10000); // Long timeout in case of tug-a-war

            // Step 2a
            pjRoutine.addStep(new AutoGrabMogoStep(mogoGrabber, false));
            pjRoutine.driveRelative(-6.0, 1500, mogoGrabSpeed);
            pjRoutine.addStep(new AutoGrabMogoStep(mogoGrabber, true));
            pjRoutine.addStep(conveyorStep);

            pjRoutine.rotateTo(M_PI * -0.355);
            pjRoutine.addStep(new AutoIntakeSetArmPositionStep(intake, IntakeSystem::ArmPosition::BOTTOM_RING));
            pjRoutine.driveRelative(22.0, 2000, intakeSpeed);
            pjRoutine.addStep(new AutoIntakeClawStep(intake, true));
            pjRoutine.driveRelative(-16);
            pjRoutine.addStep(new AutoIntakeSetArmPositionStep(intake, IntakeSystem::ArmPosition::FOURTH_RING));
            pjRoutine.driveRelative(22.0, 2000, intakeSpeed);
            pjRoutine.addStep(new AutoConveyorPickupStep(conveyor, true, 1000));
            pjRoutine.driveRelative(-10);
            pjRoutine.rotateTo(0);
            pjRoutine.driveRelative(5);
            pjRoutine.addStep(new AutoIntakeClawStep(intake, false));
            pjRoutine.addStep(new AutoIntakeSetArmPositionStep(intake, IntakeSystem::ArmPosition::INTAKE));
            pjRoutine.drive(-5);

            // pjRoutine.pause(4000); // Wait for Blaze

            // Step 2b
            // pjRoutine.addStep(new AutoGrabMogoStep(mogoGrabber, false));
            // pjRoutine.rotateTo(M_PI * -0.6);
            // pjRoutine.drive(-32);
            // pjRoutine.rotateTo(M_PI * -0.88);
            // pjRoutine.driveRelative(-10, 1000, mogoGrabSpeed);
            // pjRoutine.addStep(new AutoGrabMogoStep(mogoGrabber, true));
            // pjRoutine.driveRelative(10);
            // pjRoutine.rotateTo(M_PI * -0.58);

            // pjRoutine.addStep(new AutoGrabMogoStep(mogoGrabber, false));
            // pjRoutine.driveRelative(-6, 1000, mogoGrabSpeed);
            // pjRoutine.addStep(new AutoGrabMogoStep(mogoGrabber, true));
            // pjRoutine.drive(48.0);

            // Step 3
            pjRoutine.rotateTo(M_PI);
            pjRoutine.drive(16);
            pjRoutine.rotateTo(M_PI);
            pjRoutine.drive(12, 2000, intakeSpeed);
            pjRoutine.addStep(new AutoConveyorPickupStep(conveyor, true, 2000));

            // Step 4
            // pjRoutine.rotateTo(M_PI * -0.75);
            // pjRoutine.addStep(new AutoIntakeSetArmPositionStep(intake, IntakeSystem::ArmPosition::NEUTRAL_STAKE));
            // pjRoutine.drive(21.0, 2500, cornerSpeed);
            // pjRoutine.addStep(new AutoConveyorPickupStep(conveyor, true, 5000));
            // pjRoutine.drive(-20);

            // Step 4;
            pjRoutine.rotateTo(M_PI * 0.545);
            pjRoutine.addStep(new AutoIntakeSetArmPositionStep(intake, IntakeSystem::ArmPosition::BOTTOM_RING));
            pjRoutine.drive(35.0);

            pjRoutine.drive(18.0, 2000, mogoGrabSpeed);
            pjRoutine.addStep(new AutoIntakeClawStep(intake, true));
            pjRoutine.driveRelative(-4.0);
            pjRoutine.addStep(new AutoIntakeSetArmPositionStep(intake, IntakeSystem::ArmPosition::NEUTRAL_STAKE));
            pjRoutine.pause(200);
            pjRoutine.rotateTo(M_PI);
            pjRoutine.addStep(new AutoIntakeSetArmPositionStep(intake, IntakeSystem::ArmPosition::ALLIANCE_STAKE));
            pjRoutine.drive(-16.0);
            pjRoutine.rotateTo(M_PI);
            pjRoutine.driveRelative(16.0, 2000, intakeSpeed);
            pjRoutine.addStep(new AutoIntakeClawStep(intake, false));
            pjRoutine.driveRelative(-8.0);
            pjRoutine.addStep(new AutoIntakeClawStep(intake, true));
            pjRoutine.driveRelative(8.0);
            pjRoutine.addStep(new AutoIntakeSetArmPositionStep(intake, IntakeSystem::ArmPosition::NEUTRAL_STAKE));
            pjRoutine.addStep(new AutoIntakeClawStep(intake, false));
            pjRoutine.driveRelative(-8.0);

            // Step 5
            pjRoutine.rotateTo(M_PI * 0.5);
            pjRoutine.addStep(new AutoIntakeSetArmPositionStep(intake, IntakeSystem::ArmPosition::INTAKE));
            pjRoutine.drive(32.0);
            pjRoutine.rotateTo(M_PI * 0.3);

            return pjRoutine.build();
        }

        static AutoStepList *createBlazeMatchAuto(
            ChassisBase &chassis,
            OdomSource &odometry,
            IntakeSystem &intake,
            ConveyorSystem &conveyor,
            MogoGrabSystem &mogoGrabber,
            bool isBlue = false)
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
                0.5,  // maxSpeed
                0.15, // minSpeed
                0.05, // goalDist
                0.5,  // goalSpeed
            };

            // Speed Options
            AutoDriveToStep::Options mogoGrabSpeed = {
                drivePID,
                drivingRotatePID,
                0.25, // maxSpeed
                1.0,  // goalDist
                2.0   // goalSpeed
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
                0.15, // maxSpeed
                1.0,  // goalDist
                2.0   // goalSpeed
            };
            AutoDriveToStep::Options maxSpeed = {
                drivePID,
                rotatePID,
                0.8, // maxSpeed
                4.0, // goalDist
                6.0  // goalSpeed
            };

            AutoBuilder blazeRoutine = AutoBuilder(chassis, odometry);

            // Initial State
            blazeRoutine.useTransformer(isBlue ? new MirrorTransform() : nullptr);
            blazeRoutine.setPose(-52, 36, M_PI * 0.68888);
            blazeRoutine.addAsyncStep(new AutoIntakeMoveArmStep(intake));
            blazeRoutine.addAsyncStep(new AutoConveyorStep(conveyor, intake, mogoGrabber));
            blazeRoutine.addStep(new AutoIntakeSetArmPositionStep(intake, IntakeSystem::ArmPosition::INTAKE));
            blazeRoutine.addStep(new AutoIntakeClawStep(intake, false));
            blazeRoutine.addStep(new AutoSortStep(conveyor, isBlue ? RingType::RED : RingType::BLUE));

            // Step 1
            // blazeRoutine.drive(-24.0);
            blazeRoutine.driveRelative(-14.0, 1000, mogoGrabSpeed);
            blazeRoutine.addStep(new AutoGrabMogoStep(mogoGrabber, true));
            blazeRoutine.rotateTo(M_PI * 0.375);
            blazeRoutine.drive(40.0);
            blazeRoutine.addStep(new AutoIntakeSetArmPositionStep(intake, IntakeSystem::ArmPosition::BOTTOM_RING));
            blazeRoutine.driveRelative(12.0, 2000, intakeSpeed);
            blazeRoutine.addStep(new AutoIntakeClawStep(intake, true));
            blazeRoutine.driveRelative(-10);
            blazeRoutine.addStep(new AutoIntakeSetArmPositionStep(intake, IntakeSystem::ArmPosition::FOURTH_RING));
            blazeRoutine.driveRelative(20.0, 2000, intakeSpeed);
            blazeRoutine.addStep(new AutoConveyorPickupStep(conveyor, true, 1500));
            blazeRoutine.rotateTo(M_PI * 0.9);
            blazeRoutine.driveRelative(4);
            blazeRoutine.addStep(new AutoIntakeClawStep(intake, false));
            blazeRoutine.driveRelative(-4);

            // Step 2
            // blazeRoutine.addStep(new AutoIntakeSetArmPositionStep(intake, IntakeSystem::ArmPosition::ALLIANCE_STAKE));
            // blazeRoutine.pause(100);
            // blazeRoutine.rotateTo(M_PI * 0.25);
            // blazeRoutine.addStep(new AutoIntakeSetArmPositionStep(intake, IntakeSystem::ArmPosition::BOTTOM_RING)); // CHANGES DEPENDING ON ALLIANCE
            // blazeRoutine.driveRelative(4.5, 2000, intakeSpeed);
            // blazeRoutine.rotateTo(0);
            // blazeRoutine.driveRelative(10, 1000, intakeSpeed);
            // blazeRoutine.addStep(new AutoIntakeClawStep(intake, true));
            // blazeRoutine.driveRelative(-10);
            // blazeRoutine.addStep(new AutoIntakeClawStep(intake, false));
            // blazeRoutine.addStep(new AutoIntakeSetArmPositionStep(intake, IntakeSystem::ArmPosition::INTAKE));
            // blazeRoutine.addStep(new AutoConveyorPickupStep(conveyor, true, 1000));
            blazeRoutine.rotateTo(M_PI * 0.25);
            blazeRoutine.drive(-12.0);

            // Step 3
            blazeRoutine.rotateTo(M_PI);
            blazeRoutine.drive(19, 2000, intakeSpeed);
            blazeRoutine.addStep(new AutoConveyorPickupStep(conveyor, true, 1500));
            blazeRoutine.driveRelative(-3);

            // Step 4
            blazeRoutine.rotateTo(M_PI * 0.75);
            blazeRoutine.addStep(new AutoIntakeSetArmPositionStep(intake, IntakeSystem::ArmPosition::NEUTRAL_STAKE));
            blazeRoutine.drive(21.0, 2500, cornerSpeed);
            blazeRoutine.addStep(new AutoConveyorPickupStep(conveyor, true, 5000));
            blazeRoutine.drive(-20);

            // Step 5
            blazeRoutine.rotateTo(M_PI * -0.25);
            blazeRoutine.driveRelative(-18.0, 1000);
            blazeRoutine.addStep(new AutoIntakeClawStep(intake, false));
            blazeRoutine.addStep(new AutoGrabMogoStep(mogoGrabber, false));
            blazeRoutine.addStep(new AutoIntakeSetArmPositionStep(intake, IntakeSystem::ArmPosition::NEUTRAL_STAKE));

            // Step 6
            blazeRoutine.drive(58.0, 2000, maxSpeed);
            blazeRoutine.addStep(new AutoIntakeSetArmPositionStep(intake, IntakeSystem::ArmPosition::ALLIANCE_STAKE));

            return blazeRoutine.build();
        }

        static AutoStepList *createBlazeStartMacro(
            ChassisBase &chassis,
            OdomSource &odometry,
            IntakeSystem &intake,
            ConveyorSystem &conveyor,
            MogoGrabSystem &mogoGrabber)
        {
            // PID Params
            PIDParams drivePID = {0.15, 0.0, 10};
            PIDParams drivingRotatePID = {1.5, 0.0, 100.0};

            // Default Options
            AutoDriveToStep::Options::defaultOptions = {
                drivePID,
                drivingRotatePID,
                0.4, // maxSpeed
                2.0, // goalDist
                2.0, // goalSpeed
            };

            AutoBuilder blazeRoutine = AutoBuilder(chassis, odometry);

            // Initial State
            blazeRoutine.addAsyncStep(new AutoIntakeMoveArmStep(intake));
            blazeRoutine.addAsyncStep(new AutoConveyorStep(conveyor, intake, mogoGrabber));
            blazeRoutine.addStep(new AutoIntakeSetArmPositionStep(intake, IntakeSystem::ArmPosition::INTAKE));
            blazeRoutine.addStep(new AutoIntakeClawStep(intake, false));
            blazeRoutine.addStep(new AutoSortStep(conveyor, RingType::BLUE));

            // Mogo
            blazeRoutine.drive(12.0, 1000);
            blazeRoutine.addStep(new AutoConveyorPickupStep(conveyor, true, 500));
            blazeRoutine.drive(-11.0, 1000);
            blazeRoutine.addStep(new AutoGrabMogoStep(mogoGrabber, true));
            blazeRoutine.addStep(new AutoConveyorPickupStep(conveyor, true, 1000));
            blazeRoutine.addStep(new AutoGrabMogoStep(mogoGrabber, false));

            return blazeRoutine.build();
        }

        static AutoStepList *createBlazeSkillsAuto(
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
                0.25, // maxSpeed
                1.0,  // goalDist
                2.0   // goalSpeed
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
                0.15, // maxSpeed
                1.0,  // goalDist
                2.0   // goalSpeed
            };
            AutoDriveToStep::Options maxSpeed = {
                drivePID,
                rotatePID,
                0.8, // maxSpeed
                4.0, // goalDist
                6.0  // goalSpeed
            };

            AutoBuilder blazeRoutine = AutoBuilder(chassis, odometry);

            // Initial State
            blazeRoutine.setPose(-64, 0, 0);
            blazeRoutine.addAsyncStep(new AutoIntakeMoveArmStep(intake));
            blazeRoutine.addAsyncStep(new AutoConveyorStep(conveyor, intake, mogoGrabber));
            blazeRoutine.addStep(new AutoIntakeSetArmPositionStep(intake, IntakeSystem::ArmPosition::INTAKE));
            blazeRoutine.addStep(new AutoIntakeClawStep(intake, false));

            // Red Wall Stake
            blazeRoutine.drive(12.0, 2000, intakeSpeed);
            blazeRoutine.addStep(new AutoConveyorPickupStep(conveyor, true, 500));
            blazeRoutine.drive(-11.0, 2000, intakeSpeed);
            blazeRoutine.addStep(new AutoGrabMogoStep(mogoGrabber, true));
            blazeRoutine.addStep(new AutoConveyorPickupStep(conveyor, true, 1000));
            blazeRoutine.addStep(new AutoGrabMogoStep(mogoGrabber, false));

            // SW Mogo
            blazeRoutine.drive(17.0);
            blazeRoutine.rotateTo(M_PI * 0.25);
            blazeRoutine.drive(30.0);
            blazeRoutine.driveRelative(8, 1000, intakeSpeed);
            blazeRoutine.addStep(new AutoConveyorPickupStep(conveyor, true, 1500));
            blazeRoutine.driveRelative(-4);
            blazeRoutine.rotateTo(M_PI * -0.5);
            blazeRoutine.drive(-23.0, 2000, mogoGrabSpeed);
            blazeRoutine.addStep(new AutoGrabMogoStep(mogoGrabber, true));
            blazeRoutine.addStep(new AutoConveyorPickupStep(conveyor, true, 1000));

            blazeRoutine.rotateTo(0);
            blazeRoutine.drive(24.0, 2000, intakeSpeed);
            blazeRoutine.addStep(new AutoConveyorPickupStep(conveyor, true, 1500));

            blazeRoutine.drive(-24.0);
            blazeRoutine.rotateTo(M_PI * 0.15);
            blazeRoutine.drive(24.0, 2000, intakeSpeed);
            blazeRoutine.addStep(new AutoConveyorPickupStep(conveyor, true, 1500));
            blazeRoutine.drive(-24.0);

            blazeRoutine.rotateTo(M_PI);
            blazeRoutine.drive(20);
            blazeRoutine.driveRelative(7, 1000, intakeSpeed);
            blazeRoutine.addStep(new AutoConveyorPickupStep(conveyor, true, 1500));
            blazeRoutine.addStep(new AutoIntakeSetArmPositionStep(intake, IntakeSystem::ArmPosition::ALLIANCE_STAKE));
            blazeRoutine.rotateTo(M_PI * 0.75);
            blazeRoutine.drive(18, 1500);
            blazeRoutine.addStep(new AutoConveyorPickupStep(conveyor, true, 2000));
            blazeRoutine.drive(-10.0);
            blazeRoutine.rotateTo(M_PI * -0.25);
            blazeRoutine.addStep(new AutoIntakeSetArmPositionStep(intake, IntakeSystem::ArmPosition::INTAKE));
            blazeRoutine.addStep(new AutoGrabMogoStep(mogoGrabber, false));

            // SE Mogo
            blazeRoutine.drive(-14.5, 1000);

            blazeRoutine.drive(22.0);
            blazeRoutine.rotateTo(0);
            blazeRoutine.drive(46.0);
            blazeRoutine.rotateTo(M_PI * 0.75);
            blazeRoutine.drive(-25.0);
            blazeRoutine.drive(-12.0, 2000, mogoGrabSpeed);
            blazeRoutine.addStep(new AutoGrabMogoStep(mogoGrabber, true));

            blazeRoutine.rotateTo(M_PI * 0.5);
            blazeRoutine.drive(24.0, 2000, intakeSpeed);
            blazeRoutine.addStep(new AutoConveyorPickupStep(conveyor, true, 1500));
            blazeRoutine.rotateTo(0);
            blazeRoutine.drive(26.0, 2000, intakeSpeed);
            blazeRoutine.addStep(new AutoConveyorPickupStep(conveyor, true, 1500));
            blazeRoutine.rotateTo(M_PI * -0.5);
            blazeRoutine.drive(24.0, 2000, intakeSpeed);
            blazeRoutine.addStep(new AutoConveyorPickupStep(conveyor, true, 1500));
            blazeRoutine.drive(-24.0);

            blazeRoutine.rotateTo(M_PI * -0.75);
            blazeRoutine.addStep(new AutoGrabMogoStep(mogoGrabber, false));
            blazeRoutine.drive(-16.0, 1500);
            blazeRoutine.drive(20.0);

            // Blue Wall Stake
            blazeRoutine.rotateTo(M_PI * -0.4);

            blazeRoutine.drive(50);
            blazeRoutine.addStep(new AutoConveyorPickupStep(conveyor, true, 1500));
            blazeRoutine.rotateTo(M_PI);
            blazeRoutine.drive(16.0);
            blazeRoutine.drive(-17, 2000, intakeSpeed);
            blazeRoutine.addStep(new AutoGrabMogoStep(mogoGrabber, true));
            blazeRoutine.addStep(new AutoConveyorPickupStep(conveyor, true, 5000));

            // Climb
            // blazeRoutine.drive(16.0);
            // blazeRoutine.rotate(M_PI * 0.75);
            // blazeRoutine.drive(34.0);
            // blazeRoutine.rotate(M_PI * -0.75);
            // blazeRoutine.drive(16.0);

            return blazeRoutine.build();
        }

        static AutoStepList *createTestAuto(DummyChassis &chassis)
        {
            OdomSource &odometry = chassis;
            bool isBlue = true;

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
                0.5,  // maxSpeed
                0.15, // minSpeed
                0.05, // goalDist
                0.5,  // goalSpeed
            };

            // Speed Options
            AutoDriveToStep::Options mogoGrabSpeed = {
                drivePID,
                drivingRotatePID,
                0.25, // maxSpeed
                1.0,  // goalDist
                2.0   // goalSpeed
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
                0.15, // maxSpeed
                1.0,  // goalDist
                2.0   // goalSpeed
            };
            AutoDriveToStep::Options maxSpeed = {
                drivePID,
                rotatePID,
                0.8, // maxSpeed
                4.0, // goalDist
                6.0  // goalSpeed
            };

            AutoBuilder blazeRoutine = AutoBuilder(chassis, odometry);

            // Initial State
            blazeRoutine.useTransformer(isBlue ? new MirrorTransform() : nullptr);
            blazeRoutine.setPose(-52, 12, M_PI * 0.68888);
            // blazeRoutine.addAsyncStep(new AutoIntakeMoveArmStep(intake));
            // blazeRoutine.addAsyncStep(new AutoConveyorStep(conveyor, intake, mogoGrabber));
            // blazeRoutine.addStep(new AutoIntakeSetArmPositionStep(intake, IntakeSystem::ArmPosition::INTAKE));
            // blazeRoutine.addStep(new AutoIntakeClawStep(intake, false));

            // Step 1
            // blazeRoutine.drive(-24.0);
            blazeRoutine.driveRelative(-14.0, 1000, mogoGrabSpeed);
            // blazeRoutine.addStep(new AutoGrabMogoStep(mogoGrabber, true));
            blazeRoutine.rotateTo(M_PI * 0.375);
            blazeRoutine.drive(40.0);
            // blazeRoutine.addStep(new AutoIntakeSetArmPositionStep(intake, IntakeSystem::ArmPosition::BOTTOM_RING));
            blazeRoutine.driveRelative(12.0, 2000, intakeSpeed);
            // blazeRoutine.addStep(new AutoIntakeClawStep(intake, true));
            blazeRoutine.driveRelative(-10);
            // blazeRoutine.addStep(new AutoIntakeSetArmPositionStep(intake, IntakeSystem::ArmPosition::FOURTH_RING));
            blazeRoutine.driveRelative(20.0, 2000, intakeSpeed);
            // blazeRoutine.addStep(new AutoConveyorPickupStep(conveyor, true, 1000));
            blazeRoutine.rotateTo(M_PI * 0.9);
            blazeRoutine.driveRelative(4);
            // blazeRoutine.addStep(new AutoIntakeClawStep(intake, false));
            blazeRoutine.driveRelative(-4);
            blazeRoutine.rotateTo(M_PI * 0.25);
            blazeRoutine.drive(-17.0);

            // Step 3
            blazeRoutine.rotateTo(M_PI);
            blazeRoutine.drive(17, 2000, intakeSpeed);
            // blazeRoutine.addStep(new AutoConveyorPickupStep(conveyor, true, 1000));
            blazeRoutine.driveRelative(-3);

            // Step 4
            blazeRoutine.rotateTo(M_PI * 0.75);
            // blazeRoutine.addStep(new AutoIntakeSetArmPositionStep(intake, IntakeSystem::ArmPosition::NEUTRAL_STAKE));
            blazeRoutine.drive(21.0, 2500, cornerSpeed);
            // blazeRoutine.addStep(new AutoConveyorPickupStep(conveyor, true, 5000));
            blazeRoutine.drive(-20);

            // Step 5
            blazeRoutine.rotateTo(M_PI * -0.25);
            blazeRoutine.driveRelative(-18.0, 1000);
            // blazeRoutine.addStep(new AutoIntakeClawStep(intake, false));
            // blazeRoutine.addStep(new AutoGrabMogoStep(mogoGrabber, false));
            // blazeRoutine.addStep(new AutoIntakeSetArmPositionStep(intake, IntakeSystem::ArmPosition::NEUTRAL_STAKE));

            // Step 6
            blazeRoutine.drive(58.0, 2000, maxSpeed);
            // blazeRoutine.addStep(new AutoIntakeSetArmPositionStep(intake, IntakeSystem::ArmPosition::ALLIANCE_STAKE));

            return blazeRoutine.build();
        }
    };
}