#pragma once

#include "../devils.h"
#include "subsystems/ConveyorSystem.hpp"
#include "subsystems/IntakeSystem.hpp"

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

            odometry.useIMU(&imu);
            odometry.setTicksPerRevolution(300);
            odometry.runAsync();
        }

        void autonomous() override
        {
        }

        void opcontrol() override
        {
            while (true)
            {
                test.set(test.get() + 1);
                pros::delay(1000);
            }
        }

        void disabled() override
        {
        }

        // VEXBridge
        VEXBridge bridge = VEXBridge(11);
        VBValue<int> test = VBValue<int>("Test", 0);
        VBValue<std::string> testString = VBValue<std::string>("TestString", "Hello, World!");
        VBValue<std::vector<int>> testArray = VBValue<std::vector<int>>("TestArray", {1, 2, 3});

        // Hardware
        SmartMotorGroup leftMotors = SmartMotorGroup("LeftMotors", {-6, 7, -8, 9, -10});
        SmartMotorGroup rightMotors = SmartMotorGroup("RightMotors", {16, -17, 18, -19, 20});
        SmartMotorGroup conveyorMotors = SmartMotorGroup("ConveyorMotors", {1, -2});
        InertialSensor imu = InertialSensor("IMU", 15);

        // Subsystems
        TankChassis chassis = TankChassis(leftMotors, rightMotors);

        // Auto
        TankChassisOdom odometry = TankChassisOdom(chassis, 1.375, 11);

        // NT
        VBOdom vbOdom = VBOdom("TankOdom", odometry);

        // Renderer
        EyesRenderer eyes = EyesRenderer();
    };
}