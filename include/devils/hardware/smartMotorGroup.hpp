#pragma once
#include <string>
#include "motor.hpp"
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
         * Returns the average speed of all the motors in RPM.
         * If 1 or more motors fail to return velocity, they are ignored.
         * @return The average speed of all the motors in RPM.
         * @throws std::runtime_error if no motors returned velocity.
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