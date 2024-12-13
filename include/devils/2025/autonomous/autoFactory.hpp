#pragma once

#include "devils/devils.h"
#include "autoIntakeStep.hpp"
#include "autoMogoStep.hpp"
#include "autoWackStep.hpp"

namespace devils
{
    class AutoFactory
    {
    public:
        static AutoStepList createPJAutoRoutine(
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
                0.15,  // minSpeed
                0.02,  // goalDist
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
            AutoStepList *wobbleRoutine = new AutoStepList(
                {new AutoDriveStep(chassis, odometry, -6.0),
                 new AutoDriveStep(chassis, odometry, 6.0),
                 new AutoPauseStep(chassis, 100)},
                1);

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

                // NE Mogo
                new AutoDriveStep(chassis, odometry, 18.0), // 1
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
                new AutoDriveStep(chassis, odometry, 16.0),
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
                new AutoDriveStep(chassis, odometry, 10, slowSpeed),
                new AutoDriveStep(chassis, odometry, -10, slowSpeed),
                new AutoRotateStep(chassis, odometry, M_PI * 0.13),
                new AutoDriveStep(chassis, odometry, 14, slowSpeed),
                new AutoDriveStep(chassis, odometry, -14, slowSpeed),
                new AutoRotateStep(chassis, odometry, M_PI * 0.13),
                new AutoDriveStep(chassis, odometry, 14, slowSpeed),
                new AutoDriveStep(chassis, odometry, -14, slowSpeed),
                new AutoRotateStep(chassis, odometry, M_PI * 0.13),
                new AutoDriveStep(chassis, odometry, 14, slowSpeed),
                new AutoDriveStep(chassis, odometry, -14, slowSpeed),

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
                new AutoDriveStep(chassis, odometry, 12.0),
                new AutoDriveStep(chassis, odometry, -12.0),

                // Climb
                new AutoDriveStep(chassis, odometry, 28.0), // 68
            });
            return AbsoluteStepConverter::relativeToAbsolute(autoRoutine);
        }

        static AutoStepList createBlazeAutoRoutine(
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
                0.5,   // maxSpeed
                2.0,   // goalDist
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
                0.3,   // maxSpeed
                2.0,   // goalDist
                0.002, // goalSpeed
                2000,  // timeout
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
                new AutoPauseStep(chassis, 500),

                // Red Wall Stake
                new AutoDriveStep(chassis, odometry, 6.0, slowSpeed), // 1
                new AutoDriveStep(chassis, odometry, -6.9, slowSpeed),
                new AutoGrabMogoStep(conveyor, true),
                new AutoPauseStep(chassis, 2000),
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
                new AutoDriveStep(chassis, odometry, 51.0), // 1
                new AutoRotateToStep(chassis, odometry, M_PI),
                new AutoDriveStep(chassis, odometry, 12.0),
                new AutoDriveStep(chassis, odometry, -16.5),
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

        static AutoStepList createCenterTestRoutine(
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
                0.3,   // maxSpeed
                2.0,   // goalDist
                0.002, // goalSpeed
            };
            AutoRotateToStep::Options::defaultOptions = {
                rotatePID,
                0.5,   // maxSpeed
                0.15,  // minSpeed
                0.02,  // goalDist
                0.002, // goalSpeed
            };

            AutoStepList autoRoutine = AutoStepList({
                // Start Sequence
                new AutoJumpToStep(odometry, -66, -48, 0),
                new AutoIntakeStep(intake, 1.0),
                new AutoGrabMogoStep(conveyor, true),
                new AutoPauseStep(chassis, 500),

                // Run Test
                new AutoDriveStep(chassis, odometry, 36),
                new AutoPauseStep(chassis, 500),
                new AutoDriveStep(chassis, odometry, -6),
                new AutoRotateStep(chassis, odometry, M_PI * 0.2),
                new AutoDriveStep(chassis, odometry, 8),
                new AutoDriveStep(chassis, odometry, -4),
                new AutoDriveStep(chassis, odometry, 4),
                new AutoDriveStep(chassis, odometry, -8),
                new AutoRotateStep(chassis, odometry, M_PI * -0.6),
                new AutoDriveStep(chassis, odometry, 10),
                new AutoDriveStep(chassis, odometry, -10),
                new AutoRotateStep(chassis, odometry, M_PI * 0.13),
                new AutoDriveStep(chassis, odometry, 14),
                new AutoDriveStep(chassis, odometry, -14),
                new AutoRotateStep(chassis, odometry, M_PI * 0.13),
                new AutoDriveStep(chassis, odometry, 14),
                new AutoDriveStep(chassis, odometry, -14),
                new AutoRotateStep(chassis, odometry, M_PI * 0.13),
                new AutoDriveStep(chassis, odometry, 14),
                new AutoDriveStep(chassis, odometry, -14),
                new AutoPauseStep(chassis, 9999999),
            });
            return AbsoluteStepConverter::relativeToAbsolute(autoRoutine);
        }
    };
}