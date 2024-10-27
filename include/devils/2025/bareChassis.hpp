#pragma once

#include "../devils.h"

namespace devils
{
    /**
     * Represents the Pepper Jack robot and all of its subsystems.
     */
    struct BareChassis : public Robot
    {
        /**
         * Creates a new instance of PepperJack.
         */
        BareChassis()
        {
            
            wheelOdom.setTicksPerRevolution(TICKS_PER_REVOLUTION);
            wheelOdom.runAsync();
        }

        void autonomous() override
        {
            
        }

        void opcontrol() override
        {
            // Reset Speed
            chassis.setSpeed(1.0, 1.0);

            Pose prevPose = wheelOdom.getPose();
            double time = 0;
            double lastTime = 0;

            // Loop
            while (true)
            {
                // Take Controller Inputs
                double leftY = mainController.get_analog(ANALOG_LEFT_Y) / 127.0;
                double leftX = mainController.get_analog(ANALOG_LEFT_X) / 127.0;

                // Curve Joystick Inputs
                leftY = JoystickCurve::curve(leftY, 3.0, 0.1);
                leftX = JoystickCurve::curve(leftX, 3.0, 0.05);

                chassis.move(leftY, leftX);

                // Calculate Speed 
                time = pros::millis();
                if (time - lastTime > CHECK_INTERVAL) {
                    Pose currentPose = wheelOdom.getPose();
                    double speed = (currentPose - prevPose).magnitude() / (time - lastTime) * 1000.0;
                    Logger::info("Speed: " + std::to_string(speed) + " in/s");

                    lastTime = time;
                    prevPose = currentPose;
                }

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
        static constexpr double CHECK_INTERVAL = 200.0;                        // ms
        
        static constexpr double WHEEL_RADIUS = 1.625;                         // in
        static constexpr double WHEEL_BASE = 12.0;                            // in

        // Subsystems
        TankChassis chassis = TankChassis("BareChassis.Chassis", L_MOTOR_PORTS, R_MOTOR_PORTS);
        DifferentialWheelOdometry wheelOdom = DifferentialWheelOdometry(chassis, WHEEL_RADIUS, WHEEL_BASE);

    };
}