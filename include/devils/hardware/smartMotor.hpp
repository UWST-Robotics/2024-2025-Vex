#pragma once
#include <string>
#include <cmath>
#include "pros/motors.hpp"
#include "pros/error.h"
#include "motor.hpp"
#include "../utils/logger.hpp"
#include "../network/networkObject.hpp"
#include "../network/networkTables.hpp"

namespace devils
{
    /**
     * Represents a motor object. All events are logged.
     */
    class SmartMotor : public IMotor, private INetworkObject
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
            if (errno != 0 && LOGGING_ENABLED)
                Logger::error(name + ": motor creation failed");
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
            if (status != 1 && LOGGING_ENABLED)
                Logger::error(name + ": motor move failed");
            checkHealth();
        }

        /**
         * Stops the motor.
         * @throws std::runtime_error if the motor could not be stopped.
         */
        void stop() override
        {
            int32_t status = motor.brake();
            if (status != 1 && LOGGING_ENABLED)
                Logger::error(name + ": motor stop failed");
            checkHealth();
        }

        /**
         * Gets the current position of the motor in encoder ticks.
         *
         * \note
         * 1800 ticks/rev with 36:1 gears (red cartridge),
         * 900 ticks/rev with 18:1 gears (green cartridge),
         * 300 ticks/rev with 6:1 gears (blue cartridge)
         *
         * @return The current position of the motor in encoder ticks or `PROS_ERR_F` if the position could not be retrieved.
         * @throws std::runtime_error if the position could not be retrieved.
         */
        double getPosition() override
        {
            double position = motor.get_position();
            if (position == PROS_ERR_F && LOGGING_ENABLED)
                Logger::error(name + ": motor get position failed");
            return position;
        }

        /**
         * Returns the current speed of the motor in RPM.
         * @return The current speed of the motor in RPM or `PROS_ERR_F` if the speed could not be retrieved.
         */
        double getVelocity()
        {
            double velocity = motor.get_actual_velocity();
            if (velocity == PROS_ERR_F && LOGGING_ENABLED)
                Logger::error(name + ": motor get velocity failed");
            return velocity;
        }

        /**
         * Gets the current temperature of the motor in celsius.
         * @return The current temperature of the motor in celsius or `PROS_ERR_F` if the temperature could not be retrieved.
         */
        double getTemperature()
        {
            double temperature = motor.get_temperature();
            if (temperature == PROS_ERR_F && LOGGING_ENABLED)
                Logger::error(name + ": get temperature failed");
            return temperature;
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
            if (status != 1 && LOGGING_ENABLED)
                Logger::error(name + ": motor set brake mode failed");
        }

        void serialize() override
        {
            // Update Motor Health
            checkHealth();

            // Get Prefix
            std::string networkTableKey = NetworkTables::GetHardwareKey("vex", motor.get_port());

            // Update Network Table
            NetworkTables::UpdateValue(networkTableKey + "/name", name);
            NetworkTables::UpdateValue(networkTableKey + "/type", "SmartMotor");
            NetworkTables::UpdateValue(networkTableKey + "/temperature", std::to_string(getTemperature()));
            NetworkTables::UpdateValue(networkTableKey + "/position", std::to_string(getPosition()));
            NetworkTables::UpdateValue(networkTableKey + "/velocity", std::to_string(getVelocity()));

            if (!isConnected)
                NetworkTables::UpdateValue(networkTableKey + "/faults", "Disconnected");
            else if (isOverTemp)
                NetworkTables::UpdateValue(networkTableKey + "/faults", "Over Temperature");
            else if (isDriverFault)
                NetworkTables::UpdateValue(networkTableKey + "/faults", "Driver Fault");
            else if (isOverCurrent)
                NetworkTables::UpdateValue(networkTableKey + "/faults", "Over Current");
            else if (isDriverOverCurrent)
                NetworkTables::UpdateValue(networkTableKey + "/faults", "Driver Over Current");
            else
                NetworkTables::UpdateValue(networkTableKey + "/faults", "");
        }

    private:
        /**
         * Checks and logs the current health of the motor.
         * Updates motor health variables.
         */
        void checkHealth()
        {
            // Check if Motor is Connected
            isConnected = motor.is_installed();
            if (!isConnected && LOGGING_ENABLED)
                Logger::error(name + ": motor is not connected");

            if (!isConnected)
                return;

            // Get Motor Fault Bitmark
            uint32_t motorFaults = motor.get_faults();
            isOverTemp = motorFaults & 0x01;
            isDriverFault = motorFaults & 0x02;
            isOverCurrent = motorFaults & 0x04;
            isDriverOverCurrent = motorFaults & 0x08;

            // Log Motor Health
            if (isOverTemp && LOGGING_ENABLED)
                Logger::warn(name + ": motor is over temperature");
            else if (isDriverFault && LOGGING_ENABLED)
                Logger::warn(name + ": motor driver fault");
            else if (isOverCurrent && LOGGING_ENABLED)
                Logger::warn(name + ": motor is over current");
            else if (isDriverOverCurrent && LOGGING_ENABLED)
                Logger::warn(name + ": motor driver is over current");
        }

        static constexpr bool LOGGING_ENABLED = false;

        // Hardware
        pros::Motor motor;

        // Network Table
        std::string name;

        // Motor State
        bool isOverTemp = false;
        bool isDriverFault = false;
        bool isOverCurrent = false;
        bool isDriverOverCurrent = false;
        bool isConnected = true;
    };
}