#pragma once

#include "../devils.h"

namespace devils
{
    /**
     * Represents a prototype robot and all of its subsystems.
     */
    struct PrototypeRobot : public Robot
    {
        /**
         * Creates a new instance of PepperJack.
         */
        PrototypeRobot()
        {
            NetworkTables::Reset();
            wheelOdom.setTicksPerRevolution(TICKS_PER_REVOLUTION);
            wheelOdom.runAsync();
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
        static constexpr std::initializer_list<int8_t> L_MOTOR_PORTS = {-11, 12, 13, -14};
        static constexpr std::initializer_list<int8_t> R_MOTOR_PORTS = {1, -2, -3, 4};
        static constexpr double TICKS_PER_REVOLUTION = 300.0 * (48.0 / 36.0); // ticks
        static constexpr double WHEEL_RADIUS = 1.625;                         // in
        static constexpr double WHEEL_BASE = 12.0;                            // in

        // Subsystems
        TankChassis chassis = TankChassis("Prototype.Chassis", L_MOTOR_PORTS, R_MOTOR_PORTS);
        DifferentialWheelOdometry wheelOdom = DifferentialWheelOdometry(chassis, WHEEL_RADIUS, WHEEL_BASE);

        // Additional Network Objects
        NetworkOdom networkOdom = NetworkOdom("Prototype.Odom", wheelOdom);
    };
}