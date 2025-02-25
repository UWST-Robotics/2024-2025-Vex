#pragma once

#include "devils/devils.h"
#include "autoIntakeMoveArmStep.hpp"
#include "autoIntakeSetArmPositionStep.hpp"
#include "autoIntakeClawStep.hpp"
#include "autoMogoStep.hpp"
#include "autoConveyorStep.hpp"
#include "autoMogoBranchStep.hpp"
#include "autoConveyorPickupStep.hpp"

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
            PIDParams drivePID = {0.09, 0.0, 10};
            PIDParams rotatePID = {1.4, 0.0, 100.0};

            // Default Options
            AutoDriveToStep::Options::defaultOptions = {
                drivePID,
                rotatePID,
                0.5,   // maxSpeed
                2.0,   // goalDist
                0.002, // goalSpeed
            };
            AutoRotateToStep::Options::defaultOptions = {
                rotatePID,
                0.5,   // maxSpeed
                0.12,  // minSpeed
                0.07,  // goalDist
                0.002, // goalSpeed
            };

            // Speed Options
            AutoDriveToStep::Options highSpeed = {
                drivePID,
                rotatePID,
                1.0,  // maxSpeed
                2.0,  // goalDist
                0.002 // goalSpeed
            };
            AutoDriveToStep::Options slowSpeed = {
                drivePID,
                rotatePID,
                0.3,  // maxSpeed
                2.0,  // goalDist
                0.002 // goalSpeed
            };

            // Subroutines
            AutoStepList *wobbleRoutine = new AutoStepList({
                // Repeat the wobble routine twice
                new AutoDriveStep(chassis, odometry, -6.0),
                new AutoDriveStep(chassis, odometry, 6.0),
                new AutoDriveStep(chassis, odometry, -6.0),
                new AutoDriveStep(chassis, odometry, 6.0),
            });
            AutoStepList *grabMogoRoutine = new AutoStepList({
                // Create a slight offset to force the mogo to rotate in position
                new AutoRotateStep(chassis, odometry, M_PI * -0.05),

                // Slowly reverse to prevent bouncing
                new AutoDriveStep(chassis, odometry, -5.0),

                // Again
                new AutoRotateStep(chassis, odometry, M_PI * 0.05),
                new AutoDriveStep(chassis, odometry, -5.0),

                // Actuate the mogo intake
                new AutoGrabMogoStep(mogoGrabber, true),

                // Return to the original position
                new AutoDriveStep(chassis, odometry, 10.0),
            });

            AutoBuilder pjRoutine = AutoBuilder(chassis, odometry);
            pjRoutine.setPose(-66, -48, 0);
            // pjRoutine.addStep(new AutoIntakeStep(intake, 1.0));
            pjRoutine.pause(500); // Wait for intake to extend

            // NE Mogo
            pjRoutine.drive(18.0);
            pjRoutine.pause(500); // Wait for rings to intake
            pjRoutine.rotate(M_PI);
            pjRoutine.drive(-24.0);

            pjRoutine.addStep(grabMogoRoutine);

            pjRoutine.rotate(M_PI * -0.18);
            pjRoutine.drive(16.0);
            pjRoutine.addStep(wobbleRoutine);
            pjRoutine.drive(-16.0);

            pjRoutine.rotate(M_PI * 0.5);
            pjRoutine.drive(21.0);
            pjRoutine.rotate(M_PI * -0.75);
            pjRoutine.drive(48.0);

            pjRoutine.addStep(wobbleRoutine);

            pjRoutine.drive(-12.0);
            pjRoutine.rotate(M_PI * 0.25);
            pjRoutine.addStep(new AutoGrabMogoStep(mogoGrabber, false));
            pjRoutine.drive(4.0);
            pjRoutine.drive(-20.0);

            // Center Mogo
            pjRoutine.drive(15.0);
            pjRoutine.rotate(0);
            pjRoutine.drive(48.0);
            pjRoutine.rotate(M_PI * -0.75);
            pjRoutine.drive(-34.0);

            pjRoutine.addStep(grabMogoRoutine);

            pjRoutine.rotate(M_PI * 0.75);
            pjRoutine.drive(32, 2000, slowSpeed);
            pjRoutine.drive(-6, 1000, slowSpeed);
            pjRoutine.rotate(M_PI * 0.15);
            pjRoutine.drive(8, 1000, slowSpeed);
            pjRoutine.drive(-8, 1000, slowSpeed);
            pjRoutine.rotate(M_PI * -0.4);
            pjRoutine.drive(9, 1000, slowSpeed);
            pjRoutine.drive(-9, 1000, slowSpeed);
            pjRoutine.rotate(M_PI * 0.13);
            pjRoutine.drive(12, 1000, slowSpeed);
            pjRoutine.drive(-12, 1000, slowSpeed);
            pjRoutine.rotate(M_PI * 0.13);
            pjRoutine.drive(12, 1000, slowSpeed);
            pjRoutine.drive(-12, 1000, slowSpeed);

            pjRoutine.rotate(M_PI * -0.25);

            pjRoutine.addStep(new AutoGrabMogoStep(mogoGrabber, false));
            pjRoutine.drive(26.0);

            // NW Mogo
            pjRoutine.rotate(M_PI * -0.75);
            pjRoutine.drive(-36.0);

            pjRoutine.addStep(grabMogoRoutine);

            pjRoutine.rotate(M_PI * -0.5);
            pjRoutine.drive(24.0);
            pjRoutine.rotate(M_PI * -0.75);
            pjRoutine.drive(34.0);
            pjRoutine.rotate(0);
            pjRoutine.drive(24.0);

            pjRoutine.rotate(M_PI * 0.75);
            pjRoutine.addStep(new AutoGrabMogoStep(mogoGrabber, false));
            pjRoutine.drive(6.0);
            pjRoutine.drive(-22.0, 1000, highSpeed);

            // Climb
            pjRoutine.drive(28.0);

            return pjRoutine.build();
        }

        static AutoStepList *createPJMatchAuto(
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
                1.0, // goalDist
                2.0, // goalSpeed
            };
            AutoRotateToStep::Options::defaultOptions = {
                rotatePID,
                0.5,  // maxSpeed
                0.15, // minSpeed
                0.15, // goalDist
                0.9,  // goalSpeed
            };

            // Speed Options
            AutoDriveToStep::Options mogoGrabSpeed = {
                drivePID,
                drivingRotatePID,
                0.2, // maxSpeed
                1.0, // goalDist
                2.0  // goalSpeed
            };
            AutoDriveToStep::Options cornerSpeed = {
                drivePID,
                drivingRotatePID,
                0.3, // maxSpeed
                2.0, // goalDist
                2.0  // goalSpeed
            };

            AutoBuilder pjRoutine = AutoBuilder(chassis, odometry);

            // Initial State
            pjRoutine.setPose(-42, -36, M_PI);
            pjRoutine.addAsyncStep(new AutoIntakeMoveArmStep(intake));
            pjRoutine.addAsyncStep(new AutoConveyorStep(conveyor, intake, mogoGrabber));
            pjRoutine.addStep(new AutoIntakeSetArmPositionStep(intake, IntakeSystem::ArmPosition::INTAKE));
            pjRoutine.addStep(new AutoIntakeClawStep(intake, false));

            // Step 1
            pjRoutine.addStep(new AutoGrabMogoStep(mogoGrabber, false));
            pjRoutine.drive(-28.0);
            pjRoutine.rotateTo(M_PI * 0.85);
            pjRoutine.drive(-19.5, 2000, mogoGrabSpeed);
            pjRoutine.addStep(new AutoGrabMogoStep(mogoGrabber, true));
            pjRoutine.driveRelative(18.0, 10000); // Long timeout in case of tug-a-war

            pjRoutine.addStep(new AutoGrabMogoStep(mogoGrabber, false));
            pjRoutine.driveRelative(-6.0);
            pjRoutine.addStep(new AutoGrabMogoStep(mogoGrabber, true));
            pjRoutine.driveRelative(6.0);

            // Step 2a
            pjRoutine.rotateTo(M_PI * -0.5);
            pjRoutine.drive(10.0);
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
            pjRoutine.addStep(new AutoIntakeSetArmPositionStep(intake, IntakeSystem::ArmPosition::INTAKE));
            pjRoutine.drive(25.0);
            pjRoutine.rotateTo(M_PI * -0.75);

            // Step 4
            pjRoutine.addStep(new AutoIntakeSetArmPositionStep(intake, IntakeSystem::ArmPosition::FOURTH_RING));
            pjRoutine.drive(12.0, 3000, cornerSpeed);
            pjRoutine.addStep(new AutoIntakeClawStep(intake, true));
            pjRoutine.addStep(new AutoIntakeSetArmPositionStep(intake, IntakeSystem::ArmPosition::INTAKE));
            pjRoutine.drive(-9.0, 3000, cornerSpeed);
            pjRoutine.addStep(new AutoIntakeClawStep(intake, false));
            pjRoutine.addStep(new AutoIntakeSetArmPositionStep(intake, IntakeSystem::ArmPosition::ALLIANCE_STAKE));
            pjRoutine.pause(500);
            pjRoutine.drive(11, 3000, cornerSpeed);
            pjRoutine.pause(1000);
            pjRoutine.drive(2, 3000, cornerSpeed);
            pjRoutine.pause(1000);
            pjRoutine.drive(2, 1000, cornerSpeed);
            pjRoutine.pause(1000);
            pjRoutine.drive(-20.0, 3000, cornerSpeed);

            // Step 4;
            pjRoutine.rotateTo(M_PI * 0.585);
            pjRoutine.addStep(new AutoIntakeSetArmPositionStep(intake, IntakeSystem::ArmPosition::BOTTOM_RING));
            pjRoutine.drive(38.0);

            pjRoutine.drive(12.0, 2000, mogoGrabSpeed);
            pjRoutine.addStep(new AutoIntakeClawStep(intake, true));
            pjRoutine.pause(500);
            pjRoutine.addStep(new AutoIntakeSetArmPositionStep(intake, IntakeSystem::ArmPosition::NEUTRAL_STAKE));
            pjRoutine.pause(500);
            pjRoutine.rotateTo(M_PI);
            pjRoutine.driveRelative(-2.0);
            pjRoutine.addStep(new AutoIntakeSetArmPositionStep(intake, IntakeSystem::ArmPosition::ALLIANCE_STAKE));
            pjRoutine.pause(500);
            pjRoutine.driveRelative(-3.0);
            pjRoutine.driveRelative(4.0);
            pjRoutine.addStep(new AutoIntakeClawStep(intake, false));
            pjRoutine.driveRelative(-3.0);
            pjRoutine.addStep(new AutoIntakeClawStep(intake, true));
            pjRoutine.driveRelative(4.0);
            pjRoutine.addStep(new AutoIntakeSetArmPositionStep(intake, IntakeSystem::ArmPosition::NEUTRAL_STAKE));
            pjRoutine.pause(500);
            pjRoutine.addStep(new AutoIntakeSetArmPositionStep(intake, IntakeSystem::ArmPosition::INTAKE));
            pjRoutine.pause(500);
            pjRoutine.addStep(new AutoIntakeSetArmPositionStep(intake, IntakeSystem::ArmPosition::NEUTRAL_STAKE));
            pjRoutine.addStep(new AutoIntakeClawStep(intake, false));
            pjRoutine.pause(500);
            pjRoutine.driveRelative(-3.0);
            pjRoutine.addStep(new AutoIntakeSetArmPositionStep(intake, IntakeSystem::ArmPosition::INTAKE));

            // Step 5
            pjRoutine.rotateTo(M_PI * 0.5);
            pjRoutine.drive(50.0);
            pjRoutine.rotateTo(M_PI * 0.3);
            pjRoutine.drive(7.0);

            return pjRoutine.build();
        }

        static AutoStepList *createBlazeMatchAuto(
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
            blazeRoutine.setPose(-52, 36, M_PI * 0.68888);
            blazeRoutine.addAsyncStep(new AutoIntakeMoveArmStep(intake));
            AutoAsyncStep *asyncConveyor = blazeRoutine.addAsyncStep(new AutoConveyorStep(conveyor, intake, mogoGrabber));
            blazeRoutine.addStep(new AutoIntakeSetArmPositionStep(intake, IntakeSystem::ArmPosition::INTAKE));
            blazeRoutine.addStep(new AutoIntakeClawStep(intake, false));

            // New Step 1
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
            blazeRoutine.addStep(new AutoConveyorPickupStep(conveyor, true, 1000));
            blazeRoutine.rotateTo(M_PI * 0.9);
            blazeRoutine.driveRelative(4);
            blazeRoutine.addStep(new AutoIntakeClawStep(intake, false));
            blazeRoutine.driveRelative(-4);
            blazeRoutine.addStep(new AutoIntakeSetArmPositionStep(intake, IntakeSystem::ArmPosition::ALLIANCE_STAKE));
            blazeRoutine.pause(100);
            blazeRoutine.rotateTo(M_PI * 0.25);
            blazeRoutine.addStep(new AutoIntakeSetArmPositionStep(intake, IntakeSystem::ArmPosition::BOTTOM_RING)); // CHANGES DEPENDING ON ALLIANCE
            blazeRoutine.driveRelative(4.5, 2000, intakeSpeed);
            blazeRoutine.rotateTo(0);
            blazeRoutine.driveRelative(10, 1000, intakeSpeed);
            blazeRoutine.addStep(new AutoIntakeClawStep(intake, true));
            blazeRoutine.driveRelative(-10);
            blazeRoutine.addStep(new AutoIntakeClawStep(intake, false));
            blazeRoutine.addStep(new AutoIntakeSetArmPositionStep(intake, IntakeSystem::ArmPosition::INTAKE));
            blazeRoutine.addStep(new AutoConveyorPickupStep(conveyor, true, 1000));
            blazeRoutine.rotateTo(M_PI * 0.25);
            blazeRoutine.drive(-17.0);

            // Step 3
            blazeRoutine.rotateTo(M_PI);
            // blazeRoutine.driveRelative(8.0);
            blazeRoutine.drive(17, 2000, intakeSpeed);
            blazeRoutine.addStep(new AutoConveyorPickupStep(conveyor, true, 1000));
            blazeRoutine.driveRelative(-3);
            // blazeRoutine.driveRelative(-8);

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

        static AutoStepList *createBlazeSkillsAuto(
            ChassisBase &chassis,
            OdomSource &odometry,
            IntakeSystem &intake,
            ConveyorSystem &conveyor,
            MogoGrabSystem &mogoGrabber)
        {
            // PID Params
            PIDParams drivePID = {0.09, 0.0, 10};
            PIDParams rotatePID = {1.4, 0.0, 100.0};
            PIDParams drivingRotatePID = {2.5, 0.0, 100.0};

            // Default Options
            AutoDriveToStep::Options::defaultOptions = {
                drivePID,
                drivingRotatePID,
                0.3,   // maxSpeed
                4.0,   // goalDist
                0.002, // goalSpeed
            };
            AutoRotateToStep::Options::defaultOptions = {
                rotatePID,
                0.5,   // maxSpeed
                0.2,   // minSpeed
                0.03,  // goalDist
                0.002, // goalSpeed
            };

            // Speed Options
            AutoDriveToStep::Options highSpeed = {
                drivePID,
                drivingRotatePID,
                1.0,  // maxSpeed
                2.0,  // goalDist
                0.002 // goalSpeed
            };
            AutoDriveToStep::Options slowSpeed = {
                drivePID,
                drivingRotatePID,
                0.25,  // maxSpeed
                2.0,   // goalDist
                0.002, // goalSpeed
                1500,  // timeout
            };

            // Subroutines
            AutoStepList *wobbleRoutine = new AutoStepList({
                // Repeat the wobble routine twice
                new AutoDriveStep(chassis, odometry, -6.0),
                new AutoDriveStep(chassis, odometry, 6.0),
                new AutoDriveStep(chassis, odometry, -6.0),
                new AutoDriveStep(chassis, odometry, 6.0),
            });
            AutoStepList *grabMogoRoutine = new AutoStepList({
                // Create a slight offset to force the mogo to rotate in position
                new AutoRotateStep(chassis, odometry, M_PI * -0.07),

                // Slowly reverse to prevent bouncing
                new AutoDriveStep(chassis, odometry, -6.0),

                // Again
                new AutoRotateStep(chassis, odometry, M_PI * 0.07),
                new AutoDriveStep(chassis, odometry, -6.0),

                // Actuate the mogo intake
                new AutoGrabMogoStep(mogoGrabber, true),

                // Return to the original position
                new AutoDriveStep(chassis, odometry, 10.0),
            });

            AutoBuilder blazeRoutine = AutoBuilder(chassis, odometry);
            blazeRoutine.setPose(-64, 0, 0);
            // blazeRoutine.addStep(new AutoIntakeStep(intake, 1.0));
            blazeRoutine.addStep(new AutoConveyorStep(conveyor, intake, mogoGrabber, 1.0));

            // Red Wall Stake
            blazeRoutine.drive(6.0);
            blazeRoutine.drive(-6.9);
            blazeRoutine.addStep(new AutoGrabMogoStep(mogoGrabber, true));
            blazeRoutine.pause(1000);
            blazeRoutine.addStep(new AutoGrabMogoStep(mogoGrabber, false));

            // SW Mogo
            blazeRoutine.drive(17.0);
            blazeRoutine.rotate(M_PI * 0.25);
            blazeRoutine.drive(33.0);
            blazeRoutine.rotate(M_PI * -0.5);
            blazeRoutine.drive(-23.0);

            blazeRoutine.addStep(grabMogoRoutine);

            blazeRoutine.rotate(0);
            blazeRoutine.drive(24.0);

            blazeRoutine.drive(-24.0);
            blazeRoutine.rotate(M_PI * 0.12);
            blazeRoutine.drive(24.0);
            blazeRoutine.drive(-24.0);

            blazeRoutine.rotate(M_PI);
            blazeRoutine.drive(21);
            blazeRoutine.rotate(M_PI * 0.75);
            blazeRoutine.drive(13.5);

            blazeRoutine.addStep(wobbleRoutine);

            blazeRoutine.drive(-9.0);
            blazeRoutine.rotate(M_PI * -0.25);
            blazeRoutine.addStep(new AutoGrabMogoStep(mogoGrabber, false));

            // SE Mogo
            blazeRoutine.drive(-14.5);

            blazeRoutine.drive(17.0);
            blazeRoutine.rotate(0);
            blazeRoutine.drive(48.0);
            blazeRoutine.rotate(M_PI * 0.75);
            blazeRoutine.drive(-34.0);

            blazeRoutine.addStep(grabMogoRoutine);

            blazeRoutine.rotate(M_PI * 0.5);
            blazeRoutine.drive(24.0);
            blazeRoutine.rotate(0);
            blazeRoutine.drive(24.0);
            blazeRoutine.rotate(M_PI * -0.5);
            blazeRoutine.drive(24.0);
            blazeRoutine.drive(-24.0);

            blazeRoutine.rotate(M_PI * -0.75);
            blazeRoutine.addStep(new AutoGrabMogoStep(mogoGrabber, false));
            blazeRoutine.drive(4.0);
            blazeRoutine.drive(-16.0);
            blazeRoutine.drive(16.0);

            // Blue Wall Stake
            blazeRoutine.rotate(M_PI * -0.41);

            blazeRoutine.pause(5000); // Wait for PJ

            blazeRoutine.drive(51.0);
            blazeRoutine.rotate(M_PI);
            blazeRoutine.drive(12.0);
            blazeRoutine.drive(-16, 2000, slowSpeed);
            blazeRoutine.addStep(new AutoGrabMogoStep(mogoGrabber, true));
            blazeRoutine.pause(5000);
            blazeRoutine.addStep(new AutoGrabMogoStep(mogoGrabber, false));

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

            // PID Params
            PIDParams drivePID = {0.15, 0.0, 10};
            PIDParams rotatePID = {0.7, 0.0, 50.0};
            PIDParams drivingRotatePID = {1.5, 0.0, 100.0};

            // Default Options
            AutoDriveToStep::Options::defaultOptions = {
                drivePID,
                drivingRotatePID,
                0.5, // maxSpeed
                1.0, // goalDist
                2.0, // goalSpeed
            };
            AutoRotateToStep::Options::defaultOptions = {
                rotatePID,
                0.5,  // maxSpeed
                0.15, // minSpeed
                0.15, // goalDist
                0.9,  // goalSpeed
            };

            // Speed Options
            AutoDriveToStep::Options mogoGrabSpeed = {
                drivePID,
                drivingRotatePID,
                0.2, // maxSpeed
                1.0, // goalDist
                2.0  // goalSpeed
            };
            AutoDriveToStep::Options cornerSpeed = {
                drivePID,
                drivingRotatePID,
                0.3, // maxSpeed
                2.0, // goalDist
                2.0  // goalSpeed
            };

            AutoBuilder pjRoutine = AutoBuilder(chassis, odometry);

            // Initial State
            pjRoutine.setPose(-52, -36, M_PI);
            // pjRoutine.addAsyncStep(new AutoIntakeMoveArmStep(intake));
            // AutoAsyncStep *conveyorStep = pjRoutine.addAsyncStep(new AutoConveyorStep(conveyor, mogoGrabber));
            // pjRoutine.addStep(new AutoIntakeSetArmPositionStep(intake, IntakeSystem::ArmPosition::INTAKE));
            // pjRoutine.addStep(new AutoIntakeClawStep(intake, false));

            // Step 1
            // pjRoutine.addStep(new AutoGrabMogoStep(mogoGrabber, false));
            pjRoutine.pause(500);
            pjRoutine.drive(-28.0);
            pjRoutine.rotateTo(M_PI * 0.85);
            pjRoutine.drive(-19.5, 2000, mogoGrabSpeed);
            // pjRoutine.addStep(new AutoGrabMogoStep(mogoGrabber, true));
            pjRoutine.driveRelative(18.0, 10000); // Long timeout in case of tug-a-war

            // pjRoutine.addStep(new AutoGrabMogoStep(mogoGrabber, false));
            pjRoutine.driveRelative(-6.0);
            // pjRoutine.addStep(new AutoGrabMogoStep(mogoGrabber, true));
            pjRoutine.driveRelative(6.0);

            // Step 2a
            pjRoutine.rotateTo(M_PI * -0.5);
            pjRoutine.drive(10.0);
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
            // pjRoutine.addStep(new AutoIntakeSetArmPositionStep(intake, IntakeSystem::ArmPosition::INTAKE));
            pjRoutine.drive(25.0);
            pjRoutine.rotateTo(M_PI * -0.75);

            // Step 4
            // pjRoutine.addStep(new AutoIntakeSetArmPositionStep(intake, IntakeSystem::ArmPosition::FOURTH_RING));
            pjRoutine.drive(12.0, 3000, cornerSpeed);
            // pjRoutine.addStep(new AutoIntakeClawStep(intake, true));
            // pjRoutine.addStep(new AutoIntakeSetArmPositionStep(intake, IntakeSystem::ArmPosition::INTAKE));
            pjRoutine.drive(-9.0, 3000, cornerSpeed);
            // pjRoutine.addStep(new AutoIntakeClawStep(intake, false));
            // pjRoutine.addStep(new AutoIntakeSetArmPositionStep(intake, IntakeSystem::ArmPosition::ALLIANCE_STAKE));
            pjRoutine.pause(500);
            pjRoutine.drive(11, 3000, cornerSpeed);
            pjRoutine.pause(1000);
            pjRoutine.drive(2, 3000, cornerSpeed);
            pjRoutine.pause(1000);
            pjRoutine.drive(2, 1000, cornerSpeed);
            pjRoutine.pause(1000);
            pjRoutine.drive(-20.0, 3000, cornerSpeed);

            // Step 4;
            pjRoutine.rotateTo(M_PI * 0.585);
            // pjRoutine.addStopAsyncStep(conveyorStep);
            // pjRoutine.addStep(new AutoIntakeSetArmPositionStep(intake, IntakeSystem::ArmPosition::BOTTOM_RING));
            pjRoutine.drive(38.0);

            pjRoutine.drive(12.0, 2000, mogoGrabSpeed);
            // pjRoutine.addStep(new AutoIntakeClawStep(intake, true));
            pjRoutine.pause(500);
            // pjRoutine.addStep(new AutoIntakeSetArmPositionStep(intake, IntakeSystem::ArmPosition::NEUTRAL_STAKE));
            pjRoutine.pause(500);
            pjRoutine.rotateTo(M_PI);
            pjRoutine.driveRelative(-2.0);
            // pjRoutine.addStep(new AutoIntakeSetArmPositionStep(intake, IntakeSystem::ArmPosition::ALLIANCE_STAKE));
            pjRoutine.pause(500);
            pjRoutine.driveRelative(-3.0);
            pjRoutine.driveRelative(4.0);
            // pjRoutine.addStep(new AutoIntakeClawStep(intake, false));
            pjRoutine.driveRelative(-3.0);
            // pjRoutine.addStep(new AutoIntakeClawStep(intake, true));
            pjRoutine.driveRelative(4.0);
            // pjRoutine.addStep(new AutoIntakeSetArmPositionStep(intake, IntakeSystem::ArmPosition::NEUTRAL_STAKE));
            pjRoutine.pause(500);
            // pjRoutine.addStep(new AutoIntakeSetArmPositionStep(intake, IntakeSystem::ArmPosition::INTAKE));
            pjRoutine.pause(500);
            // pjRoutine.addStep(new AutoIntakeSetArmPositionStep(intake, IntakeSystem::ArmPosition::NEUTRAL_STAKE));
            // pjRoutine.addStep(new AutoIntakeClawStep(intake, false));
            pjRoutine.pause(500);
            pjRoutine.driveRelative(-3.0);
            // pjRoutine.addStep(new AutoIntakeSetArmPositionStep(intake, IntakeSystem::ArmPosition::INTAKE));

            // Step 5
            pjRoutine.rotateTo(M_PI * 0.5);
            // pjRoutine.addAsyncStep(new AutoConveyorStep(conveyor, mogoGrabber));
            pjRoutine.drive(50.0);
            pjRoutine.rotateTo(M_PI * 0.3);
            pjRoutine.drive(7.0);

            return pjRoutine.build();
        }
    };
}