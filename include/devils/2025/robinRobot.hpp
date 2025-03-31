#pragma once

#include "../devils.h"
#include "subsystems/ConveyorSystem.hpp"
#include "subsystems/IntakeSystem.hpp"
#include "autonomous/testAuto.hpp"

namespace devils
{
    /**
     * Represents a Robin robot and all of its subsystems.
     */
    struct RobinRobot : public Robot
    {
        /**
         * Creates a new instance of Blaze.
         */
        RobinRobot()
        {
            imu.setHeadingScale(1.013);

            // odometry.useIMU(&imu);
            // odometry.setTicksPerRevolution(300);
            // odometry.runAsync();
        }

        void autonomous() override
        {
        }

        void opcontrol() override
        {
            TestAuto::runB(chassis, odometry);
        }

        void disabled() override
        {
        }

        // VEXBridge
        VEXBridge bridge = VEXBridge(0);

        // Hardware
        SmartMotorGroup leftMotors = SmartMotorGroup("LeftMotors", {-6, 7, -8, 9, -10});
        SmartMotorGroup rightMotors = SmartMotorGroup("RightMotors", {16, -17, 18, -19, 20});
        SmartMotorGroup conveyorMotors = SmartMotorGroup("ConveyorMotors", {1, -2});
        InertialSensor imu = InertialSensor("IMU", 15);

        // Subsystems
        // TankChassis chassis = TankChassis(leftMotors, rightMotors);
        DummyChassis chassis = DummyChassis();

        // Auto
        // TankChassisOdom odometry = TankChassisOdom(chassis, 1.375, 11);
        OdomSource &odometry = chassis;

        // NT
        VBOdom vbOdom = VBOdom("TankOdom", odometry);

        // Auto Options
        RobotAutoOptions autoOptions = RobotAutoOptions();
        // Renderer
        OptionsRenderer optionsRenderer = OptionsRenderer({"Match 1", "Match 2", "Skills 1", "Skills 2"}, &autoOptions);
    };
}