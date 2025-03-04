#pragma once
#include "pros/rotation.hpp"
#include "../utils/logger.hpp"
#include "../geometry/units.hpp"
#include "structs/hardwareBase.hpp"
#include <string>

namespace devils
{
    /**
     * Represents a V5 rotational sensor.
     */
    class RotationSensor : private HardwareBase
    {
    public:
        /**
         * Creates a new Rotation Sensor.
         * @param name The name of the rotational sensor (for logging purposes)
         * @param port The port of the rotational sensor (from 1 to 21). Negative ports are reversed.
         */
        RotationSensor(
            const std::string name,
            const int8_t port)
            : HardwareBase(name, "RotationSensor", port),
              rotationSensor(port)
        {
            rotationSensor.set_position(0);
            if (errno != 0)
                reportFault("Invalid port");
        }

        /**
         * Gets the absolute angle of the sensor in radians.
         * @return The absolute angle of the sensor in radians.
         */
        double getAngle()
        {
            errno = 0;
            double angle = rotationSensor.get_position();
            if (angle == PROS_ERR)
            {
                reportFault("Get rotation sensor angle failed");
                return 0;
            }
            return Units::centidegToRad(angle);
        }

        /**
         * Gets the velocity of the sensor in radians per second.
         * @return The velocity of the sensor in radians per second.
         */
        double getVelocity()
        {
            double velocity = rotationSensor.get_velocity();
            if (velocity == PROS_ERR_F)
            {
                reportFault("Get rotation sensor velocity failed");
                return 0;
            }
            return Units::centidegToRad(velocity);
        }

        /**
         * Checks if the sensor is still connected.
         * @return True if the sensor is still connected, false otherwise.
         */
        bool isConnected()
        {
            return rotationSensor.is_installed();
        }

        /**
         * Sets the position of the sensor in centidegrees.
         * @param position The position to set the sensor to.
         */
        void setPosition(uint32_t position)
        {
            rotationSensor.set_position(position);
        }

    protected:
        void serialize() override
        {
            // Update network table values
            ntAngle.set(getAngle());
            ntVelocity.set(getVelocity());

            // Check if the sensor is still connected
            if (!rotationSensor.is_installed())
                reportFault("Disconnected");
        }

    private:
        NTValue<double> ntAngle = ntGroup.makeValue("angle", 0.0);
        NTValue<double> ntVelocity = ntGroup.makeValue("velocity", 0.0);

        pros::Rotation rotationSensor;
    };
}