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

            // Post entire trajectory to NT
            double dt = 0.1;
            for (double i = 0; i < trajectory.getDuration(); i += dt)
            {
                std::string time = std::to_string(i / dt);
                TrajectoryState state = trajectory.getStateAt(i);

                NetworkTables::updateDoubleValue("Trajectory/" + time + "/time", state.time);
                NetworkTables::updateDoubleValue("Trajectory/" + time + "/x", state.currentPose.x);
                NetworkTables::updateDoubleValue("Trajectory/" + time + "/y", state.currentPose.y);
                NetworkTables::updateDoubleValue("Trajectory/" + time + "/rotation", state.currentPose.rotation);
                NetworkTables::updateDoubleValue("Trajectory/" + time + "/velocity", state.velocity);
                NetworkTables::updateDoubleValue("Trajectory/" + time + "/acceleration", state.acceleration);
                NetworkTables::updateDoubleValue("Trajectory/" + time + "/jerk", state.jerk);
            }
        }

        void autonomous() override
        {
            autoRoutine.doStep();
            chassis.stop();
        }

        void opcontrol() override
        {
            double dt = 0.05;
            while (true)
            {
                for (double i = 0; i < trajectory.getDuration(); i += dt)
                {
                    TrajectoryState state = trajectory.getStateAt(i);

                    NetworkTables::updateDoubleValue("_poses/T/x", state.currentPose.x);
                    NetworkTables::updateDoubleValue("_poses/T/y", state.currentPose.y);
                    NetworkTables::updateDoubleValue("_poses/T/rotation", state.currentPose.rotation);
                    NetworkTables::updateDoubleValue("velocity", state.velocity);
                    NetworkTables::updateDoubleValue("acceleration", state.acceleration);
                    NetworkTables::updateDoubleValue("jerk", state.jerk);

                    pros::delay(50);
                }
            }
            // autonomous();
            //  // Loop
            //  while (true)
            //  {
            //      // Take Controller Inputs
            //      double leftY = mainController.get_analog(ANALOG_LEFT_Y) / 127.0;
            //      double leftX = mainController.get_analog(ANALOG_LEFT_X) / 127.0;

            //     // Curve Joystick Inputs
            //     leftY = JoystickCurve::curve(leftY, 3.0, 0.1);
            //     leftX = JoystickCurve::curve(leftX, 3.0, 0.05);

            //     NetworkTables::updateDoubleValue("Controls/LeftY", leftY);
            //     NetworkTables::updateDoubleValue("Controls/LeftX", leftX);

            //     // Move Chassis
            //     chassis.move(leftY, leftX);

            //     // Delay to prevent the CPU from being overloaded
            //     pros::delay(10);
            // }
        }

        void disabled() override
        {
            // Stop the robot
            chassis.stop();
        }

        // Dummy Chassis
        DummyChassis chassis = DummyChassis();

        // Test Pose
        std::vector<SplinePose> splinePoses = {
            SplinePose(0, 0, 0, 12, 12),
            SplinePose(24, 24, M_PI / 2.0, 12, 12),
            SplinePose(0, 48, M_PI, 12, 12),
            SplinePose(-24, 24, -M_PI / 2.0, 12, 12),
            SplinePose(0, 0, 0, 12, 12)};

        SplinePath path = SplinePath(splinePoses);
        TrajectoryGenerator generator = TrajectoryGenerator();
        Trajectory trajectory = generator.generateTrajectory(&path);

        NTPath networkPath = NTPath("TestPath", path);

        // Additional Network Objects
        NTOdom networkOdom = NTOdom("DummyOdom", chassis);
        // NTRobot ntRobot = NTRobot();

        // Ramming Speed
        AutoDriveToStep::Options highSpeed = {
            1.0,  // accelDist
            1.0,  // decelDist
            0.8,  // maxSpeed
            0.15, // minSpeed
            2.0,  // rotationGain
            1.0   // goalDist
        };

        AutoDriveToStep::Options slowSpeed = {
            3.0,  // accelDist
            16.0, // decelDist
            0.3,  // maxSpeed
            0.18, // minSpeed
            2.0,  // rotationGain
            0.3   // goalDist
        };

        OdomSource &odometry = chassis;
        AutoStepList autoRoutine = AutoStepList({
            // Start
            new AutoJumpToStep(odometry, -64, -48, 0),

            // Section 1
            // new AutoIntakeStep(intake, 0.5),
            new AutoDriveStep(chassis, odometry, 15.0), // 1
            new AutoRotateToStep(chassis, odometry, M_PI),
            new AutoDriveStep(chassis, odometry, -23.0),
            // new AutoGrabMogoStep(conveyor, true),
            new AutoRotateToStep(chassis, odometry, M_PI * 0.5),
            new AutoDriveStep(chassis, odometry, 23.0), // 2
            new AutoRotateToStep(chassis, odometry, M_PI * -0.75),
            new AutoDriveStep(chassis, odometry, 44.0), // 3

            new AutoPauseStep(chassis, 2000),
            new AutoDriveStep(chassis, odometry, 3.0),
            new AutoDriveStep(chassis, odometry, -3.0),
            new AutoPauseStep(chassis, 2000),

            new AutoDriveStep(chassis, odometry, -12.0),
            new AutoRotateToStep(chassis, odometry, M_PI * 0.25),
            new AutoDriveStep(chassis, odometry, -14.0),
            // new AutoGrabMogoStep(conveyor, false),

            // Section 2
            new AutoDriveStep(chassis, odometry, 10.0),
            new AutoRotateToStep(chassis, odometry, 0),
            new AutoDriveStep(chassis, odometry, 48.0),
            new AutoRotateToStep(chassis, odometry, M_PI * -0.5),
            new AutoDriveStep(chassis, odometry, 6.0),

            // Section 3
            new AutoDriveStep(chassis, odometry, -6.0),
            new AutoRotateToStep(chassis, odometry, M_PI * -0.75),
            new AutoDriveStep(chassis, odometry, -34.0),
            // new AutoGrabMogoStep(conveyor, true),
            new AutoRotateToStep(chassis, odometry, M_PI * 0.7),
            new AutoDriveStep(chassis, odometry, 34.0),

            new AutoRotateToStep(chassis, odometry, M_PI),
            new AutoDriveStep(chassis, odometry, 5.0, slowSpeed),
            new AutoRotateToStep(chassis, odometry, M_PI * -0.5),
            new AutoDriveStep(chassis, odometry, 5.0, slowSpeed),
            new AutoRotateToStep(chassis, odometry, 0),
            new AutoDriveStep(chassis, odometry, 5.0, slowSpeed),
            new AutoRotateToStep(chassis, odometry, M_PI * 0.5),
            new AutoDriveStep(chassis, odometry, 5.0, slowSpeed),
            new AutoRotateToStep(chassis, odometry, M_PI * -0.3),
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
            new AutoDriveStep(chassis, odometry, -16.0, highSpeed),
            new AutoPauseStep(chassis, 1000),
            new AutoDriveStep(chassis, odometry, 6.0),
            new AutoDriveStep(chassis, odometry, -6.0, highSpeed),
            new AutoPauseStep(chassis, 1000),
            // new AutoGrabMogoStep(conveyor, false),
            new AutoDriveStep(chassis, odometry, 24.0),
        });
    };
}