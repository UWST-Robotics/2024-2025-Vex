#pragma once
#include "pros/imu.hpp"
#include "pros/optical.hpp"
#include "pros/error.h"
#include "../utils/logger.hpp"
#include "../geometry/units.hpp"
#include "../network/networkObject.hpp"
#include "../network/networkTables.hpp"
#include <string>

namespace devils
{
    /**
     * Represents a V5 optical sensor unit.
     */
    class OpticalSensor : private INetworkObject
    {
    public:
        /**
         * Creates a new Optical Sensor.
         * @param name The name of the Optical Sensor (for logging purposes)
         * @param port The port of the Optical Sensor (from 1 to 21)
         */
        OpticalSensor(std::string name, uint8_t port)
            : name(name),
              sensor(port)
        {
            if (errno != 0 && LOGGING_ENABLED)
                Logger::error(name + ": optical sensor port is invalid");
        }

        /**
         * Gets the current proximity of the Optical Sensor.
         * @return The current proximity of the Optical Sensor as an arbitrary value from 0 to 1 where 1 is the closest.
         */
        double getProximity()
        {
            std::int32_t proximity = sensor.get_proximity();
            if (proximity == PROS_ERR && LOGGING_ENABLED)
                Logger::error(name + ": optical sensor get proximity failed");
            return proximity == PROS_ERR ? 0.0 : proximity / 255.0;
        }

        void serialize() override
        {
            // Get Prefix
            std::string networkTableKey = NetworkTables::GetHardwareKey(sensor.get_port());

            // Update Network Table
            NetworkTables::UpdateValue(networkTableKey + "/name", name);
            NetworkTables::UpdateValue(networkTableKey + "/type", "OpticalSensor");
            NetworkTables::UpdateValue(networkTableKey + "/proximity", std::to_string(getProximity()));
        }

    private:
        static constexpr bool LOGGING_ENABLED = true;

        std::string name;
        pros::Optical sensor;
    };
}