#pragma once
#include "pros/rotation.hpp"
#include "../utils/logger.hpp"
#include "../geometry/units.hpp"
#include "../network/networkObject.hpp"
#include "../network/networkTables.hpp"
#include <string>

namespace devils
{
    /**
     * Represents a V5 rotational sensor.
     */
    class RotationSensor : private INetworkObject
    {
    public:
        /**
         * Creates a new IMU.
         * @param name The name of the rotational sensor (for logging purposes)
         * @param port The port of the rotational sensor (from 1 to 21). Negative ports are reversed.
         */
        RotationSensor(std::string name, int8_t port)
            : name(name),
              rotationSensor(port)
        {
            rotationSensor.set_position(0);
            if (errno != 0 && LOGGING_ENABLED)
                Logger::error(name + ": rotationSensor port is invalid");
        }

        /**
         * Gets the absolute angle of the sensor in radians.
         * @return The absolute angle of the sensor in radians.
         */
        double getAngle()
        {
            double angle = rotationSensor.get_position();
            if (angle == PROS_ERR && LOGGING_ENABLED)
                Logger::error(name + ": rotation sensor get angle failed");
            return angle == PROS_ERR ? 0 : Units::centidegToRad(angle);
        }

        /**
         * Gets the velocity of the sensor in radians per second.
         * @return The velocity of the sensor in radians per second.
         */
        double getVelocity()
        {
            double velocity = rotationSensor.get_velocity();
            if (velocity == PROS_ERR_F && LOGGING_ENABLED)
                Logger::error(name + ": rotation sensor get velocity failed");
            return velocity == PROS_ERR_F ? 0 : Units::centidegToRad(velocity);
        }

        void serialize() override
        {
            // Get Prefix
            std::string networkTableKey = NetworkTables::GetHardwareKey("vex", rotationSensor.get_port());

            // Update Network Table
            NetworkTables::UpdateValue(networkTableKey + "/name", name);
            NetworkTables::UpdateValue(networkTableKey + "/type", "RotationSensor");
            NetworkTables::UpdateValue(networkTableKey + "/position", std::to_string(getAngle()));
            NetworkTables::UpdateValue(networkTableKey + "/velocity", std::to_string(getVelocity()));
            NetworkTables::UpdateValue(networkTableKey + "/isConnected", std::to_string(rotationSensor.is_installed()));
        }

    private:
        static constexpr bool LOGGING_ENABLED = false;

        std::string name;
        pros::Rotation rotationSensor;
    };
}