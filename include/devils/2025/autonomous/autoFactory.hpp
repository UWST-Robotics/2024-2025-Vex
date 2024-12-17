#pragma once

#include "devils/devils.h"
#include "autoIntakeStep.hpp"
#include "autoMogoStep.hpp"
#include "autoWackStep.hpp"
#include "autoConveyorStep.hpp"

namespace devils
{
    class AutoFactory
    {
    public:
        static AutoStepList *createPJAutoRoutine(
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

            AutoBuilder pjRoutine = AutoBuilder(chassis, odometry);
            pjRoutine.setPose(-66, -48, 0);
            pjRoutine.addStep(new AutoIntakeStep(intake, 1.0));
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
            pjRoutine.addStep(new AutoGrabMogoStep(conveyor, false));
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

            pjRoutine.addStep(new AutoGrabMogoStep(conveyor, false));
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
            pjRoutine.addStep(new AutoGrabMogoStep(conveyor, false));
            pjRoutine.drive(6.0);
            pjRoutine.drive(-22.0, 1000, highSpeed);

            // Climb
            pjRoutine.drive(28.0);

            return pjRoutine.build();
        }

        static AutoStepList *createBlazeAutoRoutine(
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

            AutoBuilder blazeRoutine = AutoBuilder(chassis, odometry);
            blazeRoutine.setPose(-64, 0, 0);
            blazeRoutine.addStep(new AutoIntakeStep(intake, 1.0));
            blazeRoutine.addStep(new AutoConveyorStep(conveyor, 1.0));

            // Red Wall Stake
            blazeRoutine.drive(6.0);
            blazeRoutine.drive(-6.9);
            blazeRoutine.addStep(new AutoGrabMogoStep(conveyor, true));
            blazeRoutine.pause(1000);
            blazeRoutine.addStep(new AutoGrabMogoStep(conveyor, false));

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
            blazeRoutine.addStep(new AutoGrabMogoStep(conveyor, false));

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
            blazeRoutine.addStep(new AutoGrabMogoStep(conveyor, false));
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
            blazeRoutine.addStep(new AutoGrabMogoStep(conveyor, true));
            blazeRoutine.pause(5000);
            blazeRoutine.addStep(new AutoGrabMogoStep(conveyor, false));

            // Climb
            // blazeRoutine.drive(16.0);
            // blazeRoutine.rotate(M_PI * 0.75);
            // blazeRoutine.drive(34.0);
            // blazeRoutine.rotate(M_PI * -0.75);
            // blazeRoutine.drive(16.0);

            return blazeRoutine.build();
        }

        static AutoStepList *createBlazeStartRoutine(
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
            AutoBuilder startRoutine = AutoBuilder(chassis, odometry);
            startRoutine.setPose(-64, 0, 0);
            startRoutine.addStep(new AutoIntakeStep(intake, 1.0));

            // Red Wall Stake
            startRoutine.drive(6.0);
            startRoutine.drive(-6.9);
            startRoutine.addStep(new AutoGrabMogoStep(conveyor, true));
            startRoutine.pause(1000);
            startRoutine.addStep(new AutoGrabMogoStep(conveyor, false));

            return startRoutine.build();
        }
    };
}