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
            double intakeSpeed = 0.5;

            // Loop
            while (true)
            {
                // Take Controller Inputs
                double leftY = mainController.get_analog(ANALOG_LEFT_Y) / 127.0;
                double leftX = mainController.get_analog(ANALOG_LEFT_X) / 127.0;
                double conveyor = mainController.get_analog(ANALOG_RIGHT_Y) / 127.0;

                bool intakeIn = mainController.get_digital(DIGITAL_R1);
                bool intakeOut = mainController.get_digital(DIGITAL_R2);

                bool intakeSpeedUp = mainController.get_digital_new_press(DIGITAL_UP);
                bool intakeSpeedDown = mainController.get_digital_new_press(DIGITAL_DOWN);

                // Curve Joystick Inputs
                leftY = JoystickCurve::curve(leftY, 3.0, 0.1);
                leftX = JoystickCurve::curve(leftX, 3.0, 0.05);
                conveyor = JoystickCurve::curve(conveyor, 3.0, 0.1);

                // Move Conveyor
                conveyorMotorA.moveVoltage(conveyor);
                conveyorMotorB.moveVoltage(conveyor);

                // Change Intake Speed
                if (intakeSpeedUp)
                    intakeSpeed += 0.05;
                else if (intakeSpeedDown)
                    intakeSpeed -= 0.05;

                // Print Intake Speed
                mainController.print(0, 0, "Intake Speed: %f", intakeSpeed);

                // Move Intake
                if (intakeIn)
                    intakeMotor.moveVoltage(intakeSpeed);
                else if (intakeOut)
                    intakeMotor.moveVoltage(-intakeSpeed);
                else
                    intakeMotor.stop();

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
        static constexpr std::initializer_list<int8_t> L_MOTOR_PORTS = {11, 18, -20, -12};
        static constexpr std::initializer_list<int8_t> R_MOTOR_PORTS = {-19, -14, 17, 13};
        static constexpr double TICKS_PER_REVOLUTION = 300.0 * (48.0 / 36.0); // ticks
        static constexpr double WHEEL_RADIUS = 1.625;                         // in
        static constexpr double WHEEL_BASE = 12.0;                            // in

        // Subsystems
        TankChassis chassis = TankChassis("Prototype.Chassis", L_MOTOR_PORTS, R_MOTOR_PORTS);
        DifferentialWheelOdometry wheelOdom = DifferentialWheelOdometry(chassis, WHEEL_RADIUS, WHEEL_BASE);

        SmartMotor intakeMotor = SmartMotor("Intake", 6);
        SmartMotor conveyorMotorA = SmartMotor("ConveyorA", -9);
        SmartMotor conveyorMotorB = SmartMotor("ConveyorB", 10);

        // Additional Network Objects
        NetworkService &networkService = NetworkService::getInstance();
        NetworkOdom networkOdom = NetworkOdom("Prototype.Odom", wheelOdom);
        NetworkRobotState networkRobotState = NetworkRobotState();
    };
}