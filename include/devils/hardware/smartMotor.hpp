#pragma once

#include <string>
#include <cmath>
#include "pros/motors.hpp"
#include "pros/error.h"
#include "../utils/logger.hpp"
#include "structs/motor.h"
#include "structs/hardwareBase.hpp"

namespace devils
{
    /**
     * Represents a motor object. All events are logged.
     */
    class SmartMotor : public IMotor, protected HardwareBase
    {
    public:
        /**
         * Creates a motor object.
         * @param name The name of the motor (for logging purposes)
         * @param port The port of the motor (from 1 to 21)
         * @throws std::runtime_error if the motor could not be created, likely due to an invalid port.
         */
        SmartMotor(std::string name, int8_t port)
            : HardwareBase(name, "SmartMotor", port),
              motor(port)
        {
            if (errno != 0)
                reportFault("Invalid Port");
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
                reportFault("Move Voltage Failed");
        }

        /**
         * Stops the motor.
         * @throws std::runtime_error if the motor could not be stopped.
         */
        void stop() override
        {
            int32_t status = motor.brake();
            if (status != 1)
                reportFault("Stop Failed");
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
            if (position == PROS_ERR_F)
                reportFault("Get Position Failed");
            return position;
        }

        /**
         * Sets the position of the motor in encoder ticks.
         * @param position The position to set the motor to in encoder ticks.
         */
        void setPosition(double position)
        {
            motor.set_zero_position(position);
        }

        /**
         * Returns the current speed of the motor in RPM.
         * @return The current speed of the motor in RPM or `PROS_ERR_F` if the speed could not be retrieved.
         */
        double getVelocity()
        {
            double velocity = motor.get_actual_velocity();
            if (velocity == PROS_ERR_F)
                reportFault("Get Velocity Failed");
            return velocity;
        }

        /**
         * Gets the current temperature of the motor in celsius.
         * @return The current temperature of the motor in celsius or `PROS_ERR_F` if the temperature could not be retrieved.
         */
        double getTemperature()
        {
            double temperature = motor.get_temperature();
            if (temperature == PROS_ERR_F)
                reportFault("Get Temperature Failed");
            return temperature;
        }

        /**
         * Gets the current current draw of the motor in mA.
         * @return The current current draw of the motor in mA or `PROS_ERR_F` if the current could not be retrieved.
         */
        double getCurrent()
        {
            double current = motor.get_current_draw();
            if (current == PROS_ERR_F)
                reportFault("Get Current Failed");
            return current;
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
                reportFault("Set Brake Mode Failed");
        }

    protected:
        void serialize() override
        {
            // Update temp, position, and velocity
            ntTemperature.set(getTemperature());
            ntPosition.set(getPosition());
            // ntVelocity.set(getVelocity());

            // Check if Motor is Connected
            isConnected = motor.is_installed();

            // Get Motor Fault Bitmask
            uint32_t motorFaults = motor.get_faults();
            isOverTemp = motorFaults & 0x01;
            isDriverFault = motorFaults & 0x02;
            isOverCurrent = motorFaults & 0x04;
            isDriverOverCurrent = motorFaults & 0x08;

            // Report Faults
            if (!isConnected)
                reportFault("Disconnected");
            else if (isOverTemp)
                reportFault("Over Temperature");
            else if (isDriverFault)
                reportFault("Driver Fault");
            else if (isOverCurrent)
                reportFault("Over Current");
            else if (isDriverOverCurrent)
                reportFault("Driver Over Current");
        }

    private:
        // NT
        NTValue<float> ntTemperature = ntGroup.makeValue("temperature", 0.0f);
        NTValue<float> ntPosition = ntGroup.makeValue("position", 0.0f);
        // NTValue<float> ntVelocity = ntGroup.makeValue("velocity", 0.0f);

        // Hardware
        pros::Motor motor;

        // Motor State
        bool isPortOutOfRange = false;
        bool isPortTaken = false;
        bool isOverTemp = false;
        bool isDriverFault = false;
        bool isOverCurrent = false;
        bool isDriverOverCurrent = false;
        bool isConnected = true;
    };
}