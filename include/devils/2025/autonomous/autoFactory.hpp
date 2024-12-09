#pragma once

#include "devils/devils.h"
#include "autoIntakeStep.hpp"
#include "autoMogoStep.hpp"

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
                1.0,  // goalDist
                0.001 // goalSpeed
            };

            // Create Auto Routine
            AutoStepList autoRoutine = AutoStepList({
                // Start
                new AutoJumpToStep(odometry, -64, -48, 0),

                // Section 1
                new AutoIntakeStep(intake, 1.0),
                new AutoDriveStep(chassis, odometry, 18.0), // 1
                new AutoRotateToStep(chassis, odometry, M_PI),
                new AutoDriveStep(chassis, odometry, -34.0), //, highSpeed),
                new AutoGrabMogoStep(conveyor, true),
                new AutoDriveStep(chassis, odometry, 10.0), // <-- Recenters the robot after the high speed drive
                new AutoRotateToStep(chassis, odometry, M_PI * 0.5),
                new AutoDriveStep(chassis, odometry, 23.0), // 2
                new AutoRotateToStep(chassis, odometry, M_PI * -0.75),
                new AutoDriveStep(chassis, odometry, 45.0), // 3

                new AutoPauseStep(chassis, 500),
                new AutoDriveStep(chassis, odometry, -6.0),
                new AutoDriveStep(chassis, odometry, 6.0),
                new AutoPauseStep(chassis, 500),

                new AutoDriveStep(chassis, odometry, -12.0),
                new AutoRotateToStep(chassis, odometry, M_PI * 0.25),
                new AutoDriveStep(chassis, odometry, -16.0),
                new AutoGrabMogoStep(conveyor, false),

                // Section 2
                new AutoDriveStep(chassis, odometry, 14.0),
                new AutoRotateToStep(chassis, odometry, 0),
                new AutoDriveStep(chassis, odometry, 44.0),

                new AutoDriveStep(chassis, odometry, -20.0),
                new AutoRotateToStep(chassis, odometry, M_PI * -0.15),
                new AutoDriveStep(chassis, odometry, 16.0),
                new AutoDriveStep(chassis, odometry, -16.0),
                new AutoRotateToStep(chassis, odometry, 0),
                new AutoDriveStep(chassis, odometry, 24.0),

                new AutoRotateToStep(chassis, odometry, M_PI * -0.75),
                new AutoDriveStep(chassis, odometry, -48.0),
                new AutoGrabMogoStep(conveyor, true),
                new AutoDriveStep(chassis, odometry, 14.0),
                new AutoRotateToStep(chassis, odometry, M_PI * 0.75),
                new AutoDriveStep(chassis, odometry, 28.0),

                new AutoRotateToStep(chassis, odometry, M_PI),
                new AutoDriveStep(chassis, odometry, 6.0, slowSpeed),
                new AutoRotateToStep(chassis, odometry, M_PI * 0.5),
                new AutoDriveStep(chassis, odometry, 6.0, slowSpeed),
                new AutoRotateToStep(chassis, odometry, 0),
                new AutoDriveStep(chassis, odometry, 6.0, slowSpeed),
                new AutoRotateToStep(chassis, odometry, M_PI * -0.25),
                new AutoGrabMogoStep(conveyor, false),

                // Section 4
                new AutoDriveStep(chassis, odometry, 34.0),
                new AutoRotateToStep(chassis, odometry, M_PI * -0.75),
                new AutoDriveStep(chassis, odometry, -40.0),
                new AutoGrabMogoStep(conveyor, true),
                new AutoDriveStep(chassis, odometry, 10.0),
                new AutoRotateToStep(chassis, odometry, M_PI * -0.5),
                new AutoDriveStep(chassis, odometry, 21.0),
                new AutoRotateToStep(chassis, odometry, M_PI * -0.75),
                new AutoDriveStep(chassis, odometry, 36.0),
                new AutoRotateToStep(chassis, odometry, 0),
                new AutoDriveStep(chassis, odometry, 23.0),
                new AutoRotateToStep(chassis, odometry, M_PI * 0.7),
                new AutoDriveStep(chassis, odometry, -12.0, highSpeed),
                new AutoGrabMogoStep(conveyor, false),
                new AutoDriveStep(chassis, odometry, 12.0),
                new AutoDriveStep(chassis, odometry, -18.0, highSpeed),
                new AutoDriveStep(chassis, odometry, 18.0),
                new AutoDriveStep(chassis, odometry, -18.0, highSpeed),
                new AutoDriveStep(chassis, odometry, 24.0),
            });
            return AbsoluteStepConverter::relativeToAbsolute(autoRoutine);
        }

        static AutoStepList createBlazeAutoRoutine(
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
                1.0,  // goalDist
                0.001 // goalSpeed
            };

            // Create the auto routine
            AutoStepList autoRoutine = AutoStepList({
                new AutoJumpToStep(odometry, -64, 0, 0),

                new AutoIntakeStep(intake, 1.0),
                new AutoPauseStep(chassis, 500),
                new AutoDriveStep(chassis, odometry, 17.0), // 1
                new AutoRotateToStep(chassis, odometry, M_PI * 0.25),
                new AutoDriveStep(chassis, odometry, 32.0), // 2
                new AutoRotateToStep(chassis, odometry, M_PI * -0.5),
                new AutoDriveStep(chassis, odometry, -30.0),
                new AutoGrabMogoStep(conveyor, true),
                new AutoDriveStep(chassis, odometry, 4.5),
                new AutoRotateToStep(chassis, odometry, 0),
                new AutoDriveStep(chassis, odometry, 24.0), // 3

                new AutoDriveStep(chassis, odometry, -24.0),
                new AutoRotateToStep(chassis, odometry, M_PI * 0.15),
                new AutoDriveStep(chassis, odometry, 16.0), // 4
                new AutoPauseStep(chassis, 1000),
                new AutoDriveStep(chassis, odometry, -16.0),

                new AutoRotateToStep(chassis, odometry, M_PI),
                new AutoDriveStep(chassis, odometry, 23.5), // 5
                new AutoRotateToStep(chassis, odometry, M_PI * 0.75),
                new AutoDriveStep(chassis, odometry, 12), // 6

                new AutoPauseStep(chassis, 500),
                new AutoDriveStep(chassis, odometry, -6.0),
                new AutoDriveStep(chassis, odometry, 6.0),
                new AutoPauseStep(chassis, 1500),

                new AutoDriveStep(chassis, odometry, -10.0),
                new AutoRotateToStep(chassis, odometry, M_PI * -0.25),
                new AutoGrabMogoStep(conveyor, false),
                new AutoDriveStep(chassis, odometry, 6.0),
                new AutoDriveStep(chassis, odometry, -20.0),

                new AutoDriveStep(chassis, odometry, 18.0),
                new AutoRotateToStep(chassis, odometry, 0),
                new AutoDriveStep(chassis, odometry, 46.0),
                new AutoRotateToStep(chassis, odometry, M_PI * 0.75),
                new AutoDriveStep(chassis, odometry, -40.0),
                new AutoGrabMogoStep(conveyor, true),
                new AutoDriveStep(chassis, odometry, 6.0),
                new AutoRotateToStep(chassis, odometry, M_PI * 0.5),
                new AutoDriveStep(chassis, odometry, 21.0), // 1
                new AutoRotateToStep(chassis, odometry, 0),
                new AutoDriveStep(chassis, odometry, 24.0), // 2
                new AutoRotateToStep(chassis, odometry, M_PI * -0.5),
                new AutoDriveStep(chassis, odometry, 24.0), // 3
                new AutoRotateToStep(chassis, odometry, M_PI * -0.5),
                new AutoDriveStep(chassis, odometry, -24.0),
                new AutoRotateToStep(chassis, odometry, M_PI * -0.75),
                new AutoGrabMogoStep(conveyor, false),
                new AutoDriveStep(chassis, odometry, 6.0),
                new AutoDriveStep(chassis, odometry, -19.0, highSpeed),
                new AutoDriveStep(chassis, odometry, 12.0),
                new AutoDriveStep(chassis, odometry, -14.0, highSpeed),
                new AutoDriveStep(chassis, odometry, 24.0),
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

        static AutoStepList createRotationTestRoutine(
            ChassisBase &chassis,
            OdomSource &odometry)
        {
            // PID Params
            PIDParams rotatePID = {1.4, 0.0, 100.0};
            AutoRotateToStep::Options::defaultOptions = {
                rotatePID,
                1.0,    // maxSpeed
                0.1,    // minSpeed
                0.015,  // goalDist
                0.01,   // goalSpeed
                999999, // timeout
            };

            AutoStepList autoRoutine = AutoStepList({
                new AutoRotateToStep(chassis, odometry, M_PI * 1.0),
            });
            return AbsoluteStepConverter::relativeToAbsolute(autoRoutine);
        }
    };
}