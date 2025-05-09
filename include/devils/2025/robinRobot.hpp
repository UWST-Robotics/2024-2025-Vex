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

            odometry.useIMU(&imu);
            odometry.setTicksPerRevolution(300);
            odometry.runAsync();
        }

        void autonomous() override
        {
        }

        void opcontrol() override
        {
        }

        void disabled() override
        {
        }

        // VEXBridge
        VEXBridge bridge = VEXBridge();

        // Hardware
        SmartMotorGroup leftMotors = SmartMotorGroup("LeftMotors", {-6, 7, -8, 9, -10});
        SmartMotorGroup rightMotors = SmartMotorGroup("RightMotors", {16, -17, 18, -19, 20});
        InertialSensor imu = InertialSensor("IMU", 15);

        // Subsystems
        TankChassis chassis = TankChassis(leftMotors, rightMotors);
        // DummyChassis chassis = DummyChassis();

        // Auto
        TankChassisOdom odometry = TankChassisOdom(chassis, 1.375, 11);
        // OdomSource &odometry = chassis;

        // Vision
        // DevilCV camera = DevilCV("red");
        // VisionTargetOdom ringOdom = VisionTargetOdom(
        //     odometry,
        //     std::make_shared<ICamera>(camera),
        //     DevilCV::VISION_WIDTH_FOV,
        //     100);

        // NT
        VBOdom tankOdom = VBOdom("TankOdom", odometry);
        // VBOdom ringOdom = VBOdom("RingOdom", ringOdom);

        // Auto Options
        RobotAutoOptions autoOptions = RobotAutoOptions();
        std::vector<Routine> routines = {
            {0, "Match 1", true},
            {1, "Match 2", true},
            {2, "Skills 1", false},
            {3, "Skills 2", false}};
        // Renderer
        OptionsRenderer optionsRenderer = OptionsRenderer("Robin", routines, &autoOptions);
    };
}