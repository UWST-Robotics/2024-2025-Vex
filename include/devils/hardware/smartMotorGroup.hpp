#pragma once
#include <string>
#include "structs/motor.h"
#include "smartMotor.hpp"
#include "pros/error.h"

namespace devils
{
    /**
     * Represents a set of smart motors grouped together.
     */
    class SmartMotorGroup : public IMotor
    {
    public:
        /**
         * Creates a new smart motor group.
         * @param name The name of the motor group (for logging purposes)
         * @param ports The ports of the motors in the group (from 1 to 21)
         */
        SmartMotorGroup(std::string name, std::initializer_list<int8_t> ports)
            : name(name), motors()
        {
            motors.reserve(ports.size());
            for (int8_t port : ports)
                motors.push_back(std::make_shared<SmartMotor>(getMotorName(port), port));
        }

        /**
         * Runs all the motors in voltage mode.
         * @param voltage The voltage to run the motors at, from -1 to 1.
         */
        void moveVoltage(double voltage) override
        {
            for (auto motor : motors)
                motor->moveVoltage(voltage);
        }

        /**
         * Stops all the motors.
         */
        void stop() override
        {
            for (auto motor : motors)
                motor->stop();
        }

        /**
         * Gets the average position of all the motors in encoder ticks.
         * @return The average position of all the motors in encoder ticks.
         */
        double getPosition() override
        {
            // Iterate through motors and get average position
            int motorCount = 0;
            double position = 0;
            for (auto motor : motors)
            {
                double motorPosition = motor->getPosition();

                // Skip motors that fail to return position
                if (motorPosition == PROS_ERR_F)
                    continue;

                position += motorPosition;
                motorCount++;
            }

            // Log if no motors returned position
            if (motorCount == 0)
            {
                if (LOGGING_ENABLED)
                    Logger::warn(name + ": no motors returned position");
                return 0;
            }

            // Return the mean position
            return position / motorCount;
        }

        /**
         * Sets the position of all the motors in encoder ticks.
         * @param position The position to set all the motors to in encoder ticks.
         */
        void setPosition(double position)
        {
            for (auto motor : motors)
                motor->setPosition(position);
        }

        /**
         * Returns the average speed of all the motors in RPM.
         * If 1 or more motors fail to return velocity, they are ignored.
         * @return The average speed of all the motors in RPM.
         */
        double getVelocity()
        {
            // Iterate through motors and get average speed
            int motorCount = 0;
            double speed = 0;
            for (auto motor : motors)
            {
                double motorSpeed = motor->getVelocity();

                // Skip motors that fail to return velocity
                if (motorSpeed == PROS_ERR_F)
                    continue;

                speed += motorSpeed;
                motorCount++;
            }

            // Log if no motors returned velocity
            if (motorCount == 0)
            {
                if (LOGGING_ENABLED)
                    Logger::warn(name + ": no motors returned velocity");
                return 0;
            }

            // Return the mean speed
            return speed / motorCount;
        }

        /**
         * Returns the average current of all the motors in mA.
         * If 1 or more motors fail to return current, they are ignored.
         * @return The average current of all the motors in mA.
         */
        double getCurrent()
        {
            // Iterate through motors and get average current
            int motorCount = 0;
            double current = 0;
            for (auto motor : motors)
            {
                double motorCurrent = motor->getCurrent();
                printf("Motor current: %f\n", motorCurrent);

                // Skip motors that fail to return current
                if (motorCurrent == PROS_ERR)
                    continue;

                current += motorCurrent;
                motorCount++;
            }

            // Log if no motors returned current
            if (motorCount == 0)
            {
                if (LOGGING_ENABLED)
                    Logger::warn(name + ": no motors returned current");
                return 0;
            }

            // Return the mean current
            return current / motorCount;
        }

        double getTemperature()
        {
            // Iterate through motors and get average temperature
            int motorCount = 0;
            double temperature = 0;
            for (auto motor : motors)
            {
                double motorTemperature = motor->getTemperature();

                // Skip motors that fail to return temperature
                if (motorTemperature == PROS_ERR_F)
                    continue;

                temperature += motorTemperature;
                motorCount++;
            }

            // Log if no motors returned temperature
            if (motorCount == 0)
            {
                if (LOGGING_ENABLED)
                    Logger::warn(name + ": no motors returned temperature");
                return 0;
            }

            // Return the mean temperature
            return temperature / motorCount;
        }

        /**
         * Gets the motors in the motor group.
         * @return The motors in the motor group.
         */
        std::vector<std::shared_ptr<SmartMotor>> &getMotors()
        {
            return motors;
        }

        /**
         * Gets the name of each motor in the motor group.
         */
        std::string getMotorName(int32_t port)
        {
            return name + "_" + std::to_string(port);
        }

        /**
         * Sets the brake mode of all the motors in the group.
         * @param useBrakeMode True to use brake mode, false to use coast mode.
         */
        void setBrakeMode(bool useBrakeMode)
        {
            for (auto motor : motors)
                motor->setBrakeMode(useBrakeMode);
        }

    private:
        static constexpr bool LOGGING_ENABLED = false;

        const std::string name;
        std::vector<std::shared_ptr<SmartMotor>> motors;
    };
}