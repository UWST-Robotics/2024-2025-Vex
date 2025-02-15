#pragma once

#include "devils/devils.h"
#include "autoIntakeStep.hpp"
#include "autoMogoStep.hpp"
#include "autoWackStep.hpp"
#include "autoConveyorStep.hpp"
#include "autoMogoBranchStep.hpp"

namespace devils
{
    class AutoFactory
    {
    public:
        static AutoStepList createPJSkillsAuto(
            ChassisBase &chassis,
            OdomSource &odometry,
            IntakeSystem &intake,
            ConveyorSystem &conveyor)
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
                new AutoGrabMogoStep(conveyor, true),

                // Return to the original position
                new AutoDriveStep(chassis, odometry, 10.0),
            });

            // Main Auto Routine
            AutoStepList autoRoutine = AutoStepList({
                // Start Sequence
                new AutoJumpToStep(odometry, -66, -48, 0),
                new AutoIntakeStep(intake, 1.0),
                new AutoPauseStep(chassis, 500), // Wait for intake to extend

                // NE Mogo
                new AutoDriveStep(chassis, odometry, 18.0), // 1
                new AutoPauseStep(chassis, 500),            // Wait for rings to intake
                new AutoRotateToStep(chassis, odometry, M_PI),
                new AutoDriveStep(chassis, odometry, -24.0),

                grabMogoRoutine,

                new AutoRotateToStep(chassis, odometry, M_PI * -0.18),
                new AutoDriveStep(chassis, odometry, 16.0), // 1
                wobbleRoutine,
                new AutoDriveStep(chassis, odometry, -16.0),

                new AutoRotateToStep(chassis, odometry, M_PI * 0.5),
                new AutoDriveStep(chassis, odometry, 21.0), // 2
                new AutoRotateToStep(chassis, odometry, M_PI * -0.75),
                new AutoDriveStep(chassis, odometry, 48.0), // 3

                wobbleRoutine,

                new AutoDriveStep(chassis, odometry, -12.0),
                new AutoRotateToStep(chassis, odometry, M_PI * 0.25),
                new AutoGrabMogoStep(conveyor, false),
                new AutoDriveStep(chassis, odometry, 4.0),
                new AutoDriveStep(chassis, odometry, -20.0),

                // Center Mogo
                new AutoDriveStep(chassis, odometry, 15.0),
                new AutoRotateToStep(chassis, odometry, 0),
                new AutoDriveStep(chassis, odometry, 48.0),
                new AutoRotateToStep(chassis, odometry, M_PI * -0.75),
                new AutoDriveStep(chassis, odometry, -34.0),

                grabMogoRoutine,

                new AutoRotateToStep(chassis, odometry, M_PI * 0.75),
                new AutoDriveStep(chassis, odometry, 32, slowSpeed),
                new AutoDriveStep(chassis, odometry, -6, slowSpeed),
                new AutoRotateStep(chassis, odometry, M_PI * 0.15),
                new AutoDriveStep(chassis, odometry, 8, slowSpeed),
                new AutoDriveStep(chassis, odometry, -8, slowSpeed),
                new AutoRotateStep(chassis, odometry, M_PI * -0.4),
                new AutoDriveStep(chassis, odometry, 9, slowSpeed),
                new AutoDriveStep(chassis, odometry, -9, slowSpeed),
                new AutoRotateStep(chassis, odometry, M_PI * 0.13),
                new AutoDriveStep(chassis, odometry, 12, slowSpeed),
                new AutoDriveStep(chassis, odometry, -12, slowSpeed),
                new AutoRotateStep(chassis, odometry, M_PI * 0.13),
                new AutoDriveStep(chassis, odometry, 12, slowSpeed),
                new AutoDriveStep(chassis, odometry, -12, slowSpeed),

                new AutoRotateToStep(chassis, odometry, M_PI * -0.25),

                new AutoGrabMogoStep(conveyor, false),
                new AutoDriveStep(chassis, odometry, 26.0),

                // NW Mogo
                new AutoRotateToStep(chassis, odometry, M_PI * -0.75),
                new AutoDriveStep(chassis, odometry, -36.0),

                grabMogoRoutine,

                new AutoRotateToStep(chassis, odometry, M_PI * -0.5),
                new AutoDriveStep(chassis, odometry, 24.0),
                new AutoRotateToStep(chassis, odometry, M_PI * -0.75),
                new AutoDriveStep(chassis, odometry, 34.0),
                new AutoRotateToStep(chassis, odometry, 0),
                new AutoDriveStep(chassis, odometry, 24.0),

                new AutoRotateToStep(chassis, odometry, M_PI * 0.75),
                new AutoGrabMogoStep(conveyor, false),
                new AutoDriveStep(chassis, odometry, 6.0),
                new AutoDriveStep(chassis, odometry, -22.0, highSpeed),

                // Climb
                new AutoDriveStep(chassis, odometry, 28.0), // 68
            });
            return AbsoluteStepConverter::relativeToAbsolute(autoRoutine);
        }

        static AutoStepList createTestAuto(
            ChassisBase &chassis,
            OdomSource &odometry)
        {
            // PID Params
            PIDParams drivePID = {0.15, 0.0, 10};
            PIDParams rotatePID = {1.0, 0.0, 60.0};
            PIDParams drivingRotatePID = {2.5, 0.0, 100.0};

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
                1.0,  // goalSpeed
            };

            // Subroutines
            // AutoStepList autoRoutine = AutoStepList({
            //     new AutoDriveStep(chassis, odometry, 24.0),
            //     new AutoPauseStep(chassis, 2000),
            //     new AutoDriveStep(chassis, odometry, -24.0),
            //     new AutoPauseStep(chassis, 2000),
            //     new AutoDriveStep(chassis, odometry, 24.0),
            //     new AutoPauseStep(chassis, 2000),
            //     new AutoDriveStep(chassis, odometry, -24.0),
            //     new AutoPauseStep(chassis, 2000),
            //     new AutoDriveStep(chassis, odometry, 24.0),
            //     new AutoPauseStep(chassis, 2000),
            //     new AutoDriveStep(chassis, odometry, -24.0),
            //     new AutoPauseStep(chassis, 2000),
            //     new AutoDriveStep(chassis, odometry, 24.0),
            //     new AutoPauseStep(chassis, 2000),
            //     new AutoDriveStep(chassis, odometry, -24.0),
            //     new AutoPauseStep(chassis, 2000),
            // });
            AutoStepList autoRoutine = AutoStepList({
                new AutoRotateStep(chassis, odometry, M_PI * 0.5),
                new AutoPauseStep(chassis, 2000),
                new AutoRotateStep(chassis, odometry, M_PI * 0.5),
                new AutoPauseStep(chassis, 2000),
                new AutoRotateStep(chassis, odometry, M_PI * 0.5),
                new AutoPauseStep(chassis, 2000),
                new AutoRotateStep(chassis, odometry, M_PI * 0.5),
                new AutoPauseStep(chassis, 2000),
                new AutoRotateStep(chassis, odometry, M_PI * 0.5),
                new AutoPauseStep(chassis, 2000),
                new AutoRotateStep(chassis, odometry, M_PI * 0.5),
                new AutoPauseStep(chassis, 2000),
                new AutoRotateStep(chassis, odometry, M_PI * 0.5),
                new AutoPauseStep(chassis, 2000),
                new AutoRotateStep(chassis, odometry, M_PI * 0.5),
                new AutoPauseStep(chassis, 2000),
                new AutoRotateStep(chassis, odometry, M_PI * 0.5),
                new AutoPauseStep(chassis, 2000),
                new AutoRotateStep(chassis, odometry, M_PI * 0.5),
                new AutoPauseStep(chassis, 2000),
                new AutoRotateStep(chassis, odometry, M_PI * 0.5),
                new AutoPauseStep(chassis, 2000),
                new AutoRotateStep(chassis, odometry, M_PI * 0.5),
                new AutoPauseStep(chassis, 2000),
            });

            return AbsoluteStepConverter::relativeToAbsolute(autoRoutine);
        }

        static AutoStepList createPJMatchAuto(
            ChassisBase &chassis,
            OdomSource &odometry)
        { // PID Params
            PIDParams drivePID = {0.15, 0.0, 10};
            PIDParams rotatePID = {1.0, 0.0, 60.0};
            PIDParams drivingRotatePID = {2.5, 0.0, 100.0};

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
                0.15, // goalDist
                1.0,  // goalSpeed
            };

            // Speed Options
            AutoDriveToStep::Options startingSpeed = {
                drivePID,
                drivingRotatePID,
                1.0,    // maxSpeed
                2.0,    // goalDist
                2.0,    // goalSpeed
                999999, // NEVER GIVE UP, NEVER SURRENDER
            };

            // Subroutines
            AutoStepList autoRoutine = AutoStepList({
                // Step 1
                new AutoJumpToStep(odometry, -40, -36, M_PI * 0.9),
                new AutoDriveStep(chassis, odometry, -36.0, startingSpeed),
                // Swipe Mogo
                new AutoDriveStep(chassis, odometry, 20.0, startingSpeed),
                // Grab Mogo
                // Check if we got the mogo

                // Step 2a
                new AutoRotateToStep(chassis, odometry, M_PI * -0.5),
                new AutoDriveStep(chassis, odometry, 8.0),
                // new AutoPauseStep(chassis, 5000), // Make Step 2a last the same amount of time as Step 2b

                // Step 2b
                // new AutoRotateToStep(chassis, odometry, M_PI * -0.5),
                // new AutoDriveStep(chassis, odometry, -16.0),
                // new AutoRotateToStep(chassis, odometry, M_PI * -0.75),
                // new AutoDriveStep(chassis, odometry, -24.0),
                // new AutoDriveStep(chassis, odometry, 24.0),
                // new AutoRotateToStep(chassis, odometry, M_PI * -0.5),
                // new AutoDriveStep(chassis, odometry, 24.0),

                // Step 3
                new AutoRotateToStep(chassis, odometry, M_PI),
                new AutoDriveStep(chassis, odometry, 24.0),
                new AutoRotateToStep(chassis, odometry, M_PI * -0.75),
                new AutoDriveStep(chassis, odometry, 14.0),
                new AutoDriveStep(chassis, odometry, -4.0),

                // Step 4
                new AutoRotateToStep(chassis, odometry, M_PI * 0.5),
                new AutoDriveStep(chassis, odometry, 56.0),
                new AutoRotateToStep(chassis, odometry, M_PI),
                new AutoDriveStep(chassis, odometry, 6.0),
                // Wall Stake
                new AutoDriveStep(chassis, odometry, -6.0),

                // Step 5
                new AutoRotateToStep(chassis, odometry, M_PI * 0.5),
                new AutoDriveStep(chassis, odometry, 46.0),
                new AutoRotateToStep(chassis, odometry, M_PI * 0.35),
                new AutoDriveStep(chassis, odometry, 6.0),
                new AutoRotateToStep(chassis, odometry, M_PI * 0.25),
                new AutoDriveStep(chassis, odometry, 6.0),
            });

            return AbsoluteStepConverter::relativeToAbsolute(autoRoutine);
        }

        static AutoStepList createBlazeMatchAuto(
            ChassisBase &chassis,
            OdomSource &odometry)
        {
            // PID Params
            PIDParams drivePID = {0.15, 0.0, 10};
            PIDParams rotatePID = {1.0, 0.0, 60.0};
            PIDParams drivingRotatePID = {2.5, 0.0, 100.0};

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
                0.15, // goalDist
                1.0,  // goalSpeed
            };

            // Speed Options
            AutoDriveToStep::Options startingSpeed = {
                drivePID,
                drivingRotatePID,
                1.0,    // maxSpeed
                2.0,    // goalDist
                2.0,    // goalSpeed
                999999, // NEVER GIVE UP, NEVER SURRENDER
            };

            // Subroutines
            AutoStepList autoRoutine = AutoStepList({
                // Step 1
                new AutoJumpToStep(odometry, -40, 36, M_PI * -0.9),
                new AutoDriveStep(chassis, odometry, -36.0, startingSpeed),
                // Swipe Mogo
                new AutoDriveStep(chassis, odometry, 20.0, startingSpeed),
                // Grab Mogo
                // Check if we got the mogo

                // Step 2a
                // new AutoRotateToStep(chassis, odometry, M_PI * 0.5),
                // new AutoDriveStep(chassis, odometry, 8.0),
                // new AutoRotateToStep(chassis, odometry, M_PI),
                // new AutoDriveStep(chassis, odometry, 24.0),
                // new AutoRotateToStep(chassis, odometry, M_PI * 0.08),
                // new AutoDriveStep(chassis, odometry, 40.0),
                // new AutoDriveStep(chassis, odometry, -12.0),
                // new AutoRotateToStep(chassis, odometry, M_PI),
                // new AutoDriveStep(chassis, odometry, 36.0),

                // Step 2b
                new AutoRotateToStep(chassis, odometry, M_PI * 0.34),
                new AutoDriveStep(chassis, odometry, -48.0),
                new AutoRotateToStep(chassis, odometry, M_PI * 0.5),
                new AutoDriveStep(chassis, odometry, 50.0),
                new AutoRotateToStep(chassis, odometry, 0),
                new AutoDriveStep(chassis, odometry, 24.0),
                new AutoRotateToStep(chassis, odometry, M_PI * 0.15),
                new AutoDriveStep(chassis, odometry, 20.0),
                new AutoDriveStep(chassis, odometry, -10.0),
                new AutoRotateToStep(chassis, odometry, M_PI),
                new AutoDriveStep(chassis, odometry, 38.0),

                // Step 3
                new AutoRotateToStep(chassis, odometry, M_PI * 0.75),
                new AutoRotateStep(chassis, odometry, M_PI * 0.25),
                new AutoRotateStep(chassis, odometry, M_PI * -0.25),
                new AutoRotateStep(chassis, odometry, M_PI * 0.25),
                new AutoRotateStep(chassis, odometry, M_PI * -0.25),
                new AutoRotateStep(chassis, odometry, M_PI * 0.25),
                new AutoRotateStep(chassis, odometry, M_PI * -0.25),
                new AutoDriveStep(chassis, odometry, -6.0),
                new AutoDriveStep(chassis, odometry, 6.0),
                new AutoDriveStep(chassis, odometry, -6.0),
                new AutoDriveStep(chassis, odometry, 6.0),
                new AutoDriveStep(chassis, odometry, -6.0),
                new AutoRotateStep(chassis, odometry, M_PI),
                // (Drop Mogo)

                // Step 4
                new AutoRotateToStep(chassis, odometry, 0),
                new AutoDriveStep(chassis, odometry, 36.0),
                // pounce on positive corner
            });

            return AbsoluteStepConverter::relativeToAbsolute(autoRoutine);
        }

        static AutoStepList createBlazeSkillsAuto(
            ChassisBase &chassis,
            OdomSource &odometry,
            IntakeSystem &intake,
            ConveyorSystem &conveyor,
            WackerSystem &wacker)
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
                new AutoGrabMogoStep(conveyor, true),

                // Return to the original position
                new AutoDriveStep(chassis, odometry, 10.0),
            });

            // Create the auto routine
            AutoStepList autoRoutine = AutoStepList({

                // Start Sequence
                new AutoJumpToStep(odometry, -64, 0, 0),
                new AutoIntakeStep(intake, 1.0),
                new AutoConveyorStep(conveyor, 1.0),

                // Red Wall Stake
                new AutoDriveStep(chassis, odometry, 6.0), // 1
                new AutoDriveStep(chassis, odometry, -6.9),
                new AutoGrabMogoStep(conveyor, true),
                new AutoPauseStep(chassis, 1000),
                new AutoGrabMogoStep(conveyor, false),

                // SW Mogo
                new AutoDriveStep(chassis, odometry, 17.0),
                new AutoRotateToStep(chassis, odometry, M_PI * 0.25),
                new AutoDriveStep(chassis, odometry, 33.0), // 1
                new AutoRotateToStep(chassis, odometry, M_PI * -0.5),
                new AutoDriveStep(chassis, odometry, -23.0),

                grabMogoRoutine,

                new AutoRotateToStep(chassis, odometry, 0),
                new AutoDriveStep(chassis, odometry, 24.0), // 2

                new AutoDriveStep(chassis, odometry, -24.0),
                new AutoRotateToStep(chassis, odometry, M_PI * 0.12),
                new AutoDriveStep(chassis, odometry, 24.0), // 3
                new AutoDriveStep(chassis, odometry, -24.0),

                new AutoRotateToStep(chassis, odometry, M_PI),
                new AutoDriveStep(chassis, odometry, 21), // 4
                new AutoRotateToStep(chassis, odometry, M_PI * 0.75),
                new AutoDriveStep(chassis, odometry, 13.5), // 5

                wobbleRoutine,

                new AutoDriveStep(chassis, odometry, -9.0),
                new AutoRotateToStep(chassis, odometry, M_PI * -0.25),
                new AutoGrabMogoStep(conveyor, false),

                // SE Mogo
                new AutoDriveStep(chassis, odometry, -14.5),

                new AutoDriveStep(chassis, odometry, 17.0),
                new AutoRotateToStep(chassis, odometry, 0),
                new AutoDriveStep(chassis, odometry, 48.0),
                new AutoRotateToStep(chassis, odometry, M_PI * 0.75),
                new AutoDriveStep(chassis, odometry, -34.0),

                grabMogoRoutine,

                new AutoRotateToStep(chassis, odometry, M_PI * 0.5),
                new AutoDriveStep(chassis, odometry, 24.0), // 1
                new AutoRotateToStep(chassis, odometry, 0),
                new AutoDriveStep(chassis, odometry, 24.0), // 2
                new AutoRotateToStep(chassis, odometry, M_PI * -0.5),
                new AutoDriveStep(chassis, odometry, 24.0), // 3
                new AutoDriveStep(chassis, odometry, -24.0),

                new AutoRotateToStep(chassis, odometry, M_PI * -0.75),
                new AutoGrabMogoStep(conveyor, false),
                new AutoDriveStep(chassis, odometry, 4.0),
                new AutoDriveStep(chassis, odometry, -16.0),
                new AutoDriveStep(chassis, odometry, 16.0),

                // Blue Wall Stake
                new AutoRotateToStep(chassis, odometry, M_PI * -0.41),

                new AutoPauseStep(chassis, 5000), // Wait for PJ

                new AutoDriveStep(chassis, odometry, 51.0), // 1
                new AutoRotateToStep(chassis, odometry, M_PI),
                new AutoDriveStep(chassis, odometry, 12.0),
                new AutoDriveStep(chassis, odometry, -16, slowSpeed),
                new AutoGrabMogoStep(conveyor, true),
                new AutoPauseStep(chassis, 5000),
                new AutoGrabMogoStep(conveyor, false),

                // Climb
                // new AutoDriveStep(chassis, odometry, 16.0),
                // new AutoRotateToStep(chassis, odometry, M_PI * 0.75),
                // new AutoDriveStep(chassis, odometry, 34.0),
                // new AutoRotateToStep(chassis, odometry, M_PI * -0.75),
                // new AutoDriveStep(chassis, odometry, 16.0),
            });
            return AbsoluteStepConverter::relativeToAbsolute(autoRoutine);
        }

        static AutoStepList createTranslationTestRoutine(
            ChassisBase &chassis,
            OdomSource &odometry)
        {
            // PID Params
            PIDParams drivePID = {0.09, 0.0, 10};
            PIDParams rotatePID = {0.5, 0.0, 0.0};
            AutoDriveToStep::Options::defaultOptions = {
                drivePID,
                rotatePID,
                0.7,   // maxSpeed
                1.5,   // goalDist
                0.05,  // goalSpeed
                999999 // timeout
            };

            AutoStepList autoRoutine = AutoStepList({
                new AutoDriveStep(chassis, odometry, 48.0),
            });
            return AbsoluteStepConverter::relativeToAbsolute(autoRoutine);
        }

        static AutoStepList createBlazeStartRoutine(
            ChassisBase &chassis,
            OdomSource &odometry,
            IntakeSystem &intake,
            ConveyorSystem &conveyor)
        {
            // PID Params
            PIDParams drivePID = {0.09, 0.0, 10};
            PIDParams drivingRotatePID = {2.5, 0.0, 100.0};

            // Default Options
            AutoDriveToStep::Options::defaultOptions = {
                drivePID,
                drivingRotatePID,
                0.25,  // maxSpeed
                2.0,   // goalDist
                0.002, // goalSpeed
                1000,  // timeout
            };

            // Create the auto routine
            AutoStepList autoRoutine = AutoStepList({

                // Start Sequence
                new AutoJumpToStep(odometry, -64, 0, 0),
                new AutoIntakeStep(intake, 1.0),

                // Red Wall Stake
                new AutoDriveStep(chassis, odometry, 6.0), // 1
                new AutoDriveStep(chassis, odometry, -6.9),
                new AutoGrabMogoStep(conveyor, true),
                new AutoPauseStep(chassis, 1000),
                new AutoGrabMogoStep(conveyor, false),
            });
            return AbsoluteStepConverter::relativeToAbsolute(autoRoutine);
        }
    };
}