#pragma once

#include "../devils.h"

namespace devils
{
    /**
     * Represents a prototype robot and all of its subsystems.
     */
    struct TestbedRobot : public Robot
    {
        /**
         * Creates a new instance of PepperJack.
         */
        TestbedRobot()
        {
            NetworkTables::Reset();

            wheelOdom.setTicksPerRevolution(TICKS_PER_REVOLUTION);
            wheelOdom.runAsync();

            networkOdom.setSize(18.0, 18.0);
        }

        void autonomous() override
        {
        }

        void opcontrol() override
        {
            // Loop
            while (true)
            {
                // Take Controller Inputs
                double leftY = mainController.get_analog(ANALOG_LEFT_Y) / 127.0;
                double leftX = mainController.get_analog(ANALOG_LEFT_X) / 127.0;

                // Curve Joystick Inputs
                leftY = JoystickCurve::curve(leftY, 3.0, 0.1);
                leftX = JoystickCurve::curve(leftX, 3.0, 0.05);

                // Move Chassis
                chassis.move(leftY, leftX);

                // Delay to prevent the CPU from being overloaded
                pros::delay(10);
            }
        }

        void disabled() override
        {
            // Stop the robot
            chassis.stop();
        }

        // V5 Ports
        static constexpr std::initializer_list<int8_t> L_MOTOR_PORTS = {11};
        static constexpr std::initializer_list<int8_t> R_MOTOR_PORTS = {12};
        static constexpr double TICKS_PER_REVOLUTION = 300.0 * (48.0 / 36.0); // ticks
        static constexpr double WHEEL_RADIUS = 1.625;                         // in
        static constexpr double WHEEL_BASE = 12.0;                            // in

        // Subsystems
        TankChassis chassis = TankChassis("Chassis", L_MOTOR_PORTS, R_MOTOR_PORTS);
        DifferentialWheelOdometry wheelOdom = DifferentialWheelOdometry(chassis, WHEEL_RADIUS, WHEEL_BASE);

        RotationSensor rotationSensor = RotationSensor("RotationSensor", 5);
        // IMU imu = IMU("IMU", 9);
        // OpticalSensor opticalSensor = OpticalSensor("OpticalSensor", 8);
        // VisionSensor visionSensor = VisionSensor("VisionSensor", 7);
        // ScuffPneumatic scuffPneumatic = ScuffPneumatic("ScuffPneumatic", 6);

        // Additional Network Objects
        NetworkService &networkService = NetworkService::getInstance();
        NetworkOdom networkOdom = NetworkOdom("Odom", wheelOdom);
        NetworkRobotState networkRobotState = NetworkRobotState();
    };
}