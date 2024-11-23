#pragma once

#include "../devils.h"

namespace devils
{
    /**
     * Represents a debugging robot and all of its subsystems.
     * Designed to be used without any physical hardware.
     */
    struct DebugRobot : public Robot
    {
        /**
         * Creates a new instance of the debug robot.
         */
        DebugRobot()
        {
            // Reset Network Tables
            networkOdom.setSize(18.0, 18.0);
        }

        void autonomous() override
        {
            absoluteRoutine.doStep();
            chassis.stop();
        }

        void opcontrol() override
        {
            autonomous();
        }

        void disabled() override
        {
            // Stop the robot
            chassis.stop();
        }

        // Dummy Chassis
        DummyChassis chassis = DummyChassis();

        // Additional Network Objects
        NTOdom networkOdom = NTOdom("DummyOdom", chassis);
        // NTPath networkPath = NTPath("TestPath", path);

        // Autonomous Constants
        AutoDriveToStep::Options highSpeed = {
            1.0,  // accelDist
            1.0,  // decelDist
            0.8,  // maxSpeed
            0.15, // minAccelSpeed
            0.1,  // minDecelSpeed
            2.0,  // rotationGain
            1.0   // goalDist
        };

        AutoDriveToStep::Options slowSpeed = {
            3.0,  // accelDist
            16.0, // decelDist
            0.3,  // maxSpeed
            0.18, // minSpeed
            0.1,  // minDecelSpeed
            2.0,  // rotationGain
            0.3   // goalDist
        };

        // Autonomous
        OdomSource &odometry = chassis;
        AutoStepList autoRoutine = AutoStepList({
            // Start
            new AutoJumpToStep(odometry, -64, -48, 0),

            // Section 1
            // new AutoIntakeStep(intake, 1.0),
            new AutoDriveStep(chassis, odometry, 15.0), // 1
            new AutoRotateToStep(chassis, odometry, M_PI),
            new AutoDriveStep(chassis, odometry, -29.0),
            // new AutoGrabMogoStep(conveyor, true),
            new AutoDriveStep(chassis, odometry, 6.0),
            new AutoRotateToStep(chassis, odometry, M_PI * 0.5),
            new AutoDriveStep(chassis, odometry, 23.0), // 2
            new AutoRotateToStep(chassis, odometry, M_PI * -0.75),
            new AutoDriveStep(chassis, odometry, 45.0), // 3

            new AutoPauseStep(chassis, 1000),
            new AutoDriveStep(chassis, odometry, -4.0),
            new AutoDriveStep(chassis, odometry, 4.0),
            new AutoPauseStep(chassis, 1000),

            new AutoDriveStep(chassis, odometry, -11.0),
            new AutoRotateToStep(chassis, odometry, M_PI * 0.25),
            new AutoDriveStep(chassis, odometry, -12.0),
            // new AutoGrabMogoStep(conveyor, false),

            // Section 2
            new AutoDriveStep(chassis, odometry, 12.0),
            new AutoRotateToStep(chassis, odometry, 0),
            new AutoDriveStep(chassis, odometry, 44.0),

            new AutoDriveStep(chassis, odometry, -20.0),
            new AutoRotateToStep(chassis, odometry, M_PI * -0.15),
            new AutoDriveStep(chassis, odometry, 18.0),
            new AutoDriveStep(chassis, odometry, -18.0),
            new AutoRotateToStep(chassis, odometry, 0),
            new AutoDriveStep(chassis, odometry, 24.0),

            new AutoRotateToStep(chassis, odometry, M_PI * -0.75),
            new AutoDriveStep(chassis, odometry, -34.0),
            // new AutoGrabMogoStep(conveyor, true),
            new AutoRotateToStep(chassis, odometry, M_PI * 0.75),
            new AutoDriveStep(chassis, odometry, 28.0),

            new AutoRotateToStep(chassis, odometry, M_PI),
            new AutoDriveStep(chassis, odometry, 5.0, slowSpeed),
            new AutoRotateToStep(chassis, odometry, M_PI * 0.5),
            new AutoDriveStep(chassis, odometry, 5.0, slowSpeed),
            new AutoRotateToStep(chassis, odometry, 0),
            new AutoDriveStep(chassis, odometry, 5.0, slowSpeed),
            new AutoRotateToStep(chassis, odometry, M_PI * -0.25),
            // new AutoGrabMogoStep(conveyor, false),

            // Section 4
            new AutoDriveStep(chassis, odometry, 34.0),
            new AutoRotateToStep(chassis, odometry, M_PI * -0.75),
            new AutoDriveStep(chassis, odometry, -33.0),
            // new AutoGrabMogoStep(conveyor, true),
            new AutoRotateToStep(chassis, odometry, M_PI * -0.5),
            new AutoDriveStep(chassis, odometry, 24.0),
            new AutoRotateToStep(chassis, odometry, M_PI * -0.75),
            new AutoDriveStep(chassis, odometry, 32.0),
            new AutoRotateToStep(chassis, odometry, 0),
            new AutoDriveStep(chassis, odometry, 23.0),
            new AutoRotateToStep(chassis, odometry, M_PI * 0.75),
            new AutoDriveStep(chassis, odometry, -12.0, highSpeed),
            new AutoPauseStep(chassis, 1000),
            new AutoDriveStep(chassis, odometry, 6.0),
            new AutoDriveStep(chassis, odometry, -6.0, highSpeed),
            new AutoPauseStep(chassis, 1000),
            // new AutoGrabMogoStep(conveyor, false),
            new AutoDriveStep(chassis, odometry, 24.0),
        });
        AutoStepList absoluteRoutine = AbsoluteStepConverter::relativeToAbsolute(autoRoutine);

        EyesRenderer eyes = EyesRenderer();
    };
}