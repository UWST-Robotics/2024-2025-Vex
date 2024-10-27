#pragma once
#include "pros/motors.hpp"
#include "motor.hpp"
#include "../utils/logger.hpp"
#include <string>

namespace devils
{
    /**
     * Represents a motor object. All events are logged.
     */
    class SmartMotor : public IMotor
    {
    public:
        /**
         * Creates a motor object.
         * @param name The name of the motor (for logging purposes)
         * @param port The port of the motor (from 1 to 21)
         * @throws std::runtime_error if the motor could not be created, likely due to an invalid port.
         */
        SmartMotor(std::string name, int8_t port)
            : name(name),
              motor(port)
        {
            if (errno != 0)
                throw std::runtime_error(name + ": motor creation failed");
        }

        /**
         * Runs the motor in voltage mode.
         * @param voltage The voltage to run the motor at, from -1 to 1.
         * @throws std::runtime_error if the motor could not be moved.
         */
        void moveVoltage(double voltage) override
        {
            // Move Motor
            int32_t status = motor.move(voltage * 127);
            if (status != 1)
                throw std::runtime_error(name + ": motor move failed");
            _checkHealth();
        }

        /**
         * Stops the motor.
         * @throws std::runtime_error if the motor could not be stopped.
         */
        void stop() override
        {
            int32_t status = motor.brake();
            if (status != 1)
                throw std::runtime_error(name + ": motor stop failed");
            _checkHealth();
        }

        /**
         * Gets the current position of the motor in encoder ticks.
         * 
         * \note
         * 1800 ticks/rev with 36:1 gears (red cartridge), 
         * 900 ticks/rev with 18:1 gears (green cartridge),
         * 300 ticks/rev with 6:1 gears (blue cartridge)
         * 
         * @return The current position of the motor in encoder ticks.
         * @throws std::runtime_error if the position could not be retrieved.
         */
        double getPosition() override
        {
            double position = motor.get_position();
            if (position == PROS_ERR_F)
                throw std::runtime_error(name + ": motor get position failed");
            return position;
        }

        /**
         * Returns the current speed of the motor in RPM.
         * @return The current speed of the motor in RPM.
         * @throws std::runtime_error if the speed could not be retrieved.
         */
        double getVelocity()
        {
            double velocity = motor.get_actual_velocity();
            if (velocity == PROS_ERR_F)
                throw std::runtime_error(name + ": motor get velocity failed");
            return velocity;
        }

        /**
         * Gets the current temperature of the motor in celsius.
         * @return The current temperature of the motor in celsius.
         * @throws std::runtime_error if the temperature could not be retrieved.
         */
        double getTemperature()
        {
            double temperature = motor.get_temperature();
            if (temperature == PROS_ERR_F)
                throw std::runtime_error(name + ": get temperature failed");
            return temperature == PROS_ERR_F ? PROS_ERR_F : temperature;
        }

        /**
         * Checks and logs the current health of the motor. Should be called after every motor command (move, stop, etc).
         */
        void _checkHealth()
        {
            int32_t isOverTemp = motor.is_over_temp();
            int32_t isOverCurrent = motor.is_over_current();

            if ((isOverTemp == PROS_ERR || isOverCurrent == PROS_ERR) && LOGGING_ENABLED)
                Logger::warn(name + ": motor health check failed");
            else if (isOverTemp == 1 && LOGGING_ENABLED)
                Logger::warn(name + ": motor is over temperature");
            else if (isOverCurrent == 1 && LOGGING_ENABLED)
                Logger::warn(name + ": motor is over current");
        }

        /**
         * Sets the brake mode of the motor.
         *
         * \note
         * Brake mode will use the motor's e-brake to stop the motor when `stop()` is called.
         * Coast mode will allow the motor to coast to a stop when `stop()` is called.
         *
         * @param useBrakeMode True to use brake mode, false to coast mode.
         * @throws std::runtime_error if the brake mode could not be set.
         */
        void setBrakeMode(bool useBrakeMode)
        {
            int32_t status = motor.set_brake_mode(useBrakeMode ? pros::E_MOTOR_BRAKE_BRAKE : pros::E_MOTOR_BRAKE_COAST);
            if (status != 1)
                throw std::runtime_error(name + ": motor set brake mode failed");
        }

    private:
        static constexpr bool LOGGING_ENABLED = false;

        double currentVoltage = 0;
        std::string name;
        pros::Motor motor;
    };
}