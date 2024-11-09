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
            return proximity == PROS_ERR ? PROS_ERR : (proximity / 255.0);
        }

        /**
         * Gets the current hue of the Optical Sensor.
         * @return The current hue of the Optical Sensor as a value from 0 to 360.
         */
        double getHue()
        {
            std::int32_t hue = sensor.get_hue();
            if (hue == PROS_ERR && LOGGING_ENABLED)
                Logger::error(name + ": optical sensor get hue failed");
            return hue == PROS_ERR ? PROS_ERR : hue;
        }

        /**
         * Gets the current saturation of the Optical Sensor.
         * @return The current saturation of the Optical Sensor as an percentage value from 0 to 1.
         */
        double getSaturation()
        {
            std::int32_t saturation = sensor.get_saturation();
            if (saturation == PROS_ERR && LOGGING_ENABLED)
                Logger::error(name + ": optical sensor get saturation failed");
            return saturation == PROS_ERR ? PROS_ERR : saturation;
        }

        /**
         * Gets the current brightness of the Optical Sensor.
         * @return The current brightness of the Optical Sensor as an percentage value from 0 to 1.
         */
        double getBrightness()
        {
            std::int32_t brightness = sensor.get_brightness();
            if (brightness == PROS_ERR && LOGGING_ENABLED)
                Logger::error(name + ": optical sensor get brightness failed");
            return brightness == PROS_ERR ? PROS_ERR : brightness;
        }

        void serialize() override
        {
            // Get Prefix
            std::string networkTableKey = NetworkTables::GetHardwareKey("vex", sensor.get_port());

            // Update Network Table
            NetworkTables::UpdateValue(networkTableKey + "/name", name);
            NetworkTables::UpdateValue(networkTableKey + "/type", "OpticalSensor");
            NetworkTables::UpdateValue(networkTableKey + "/proximity", std::to_string(getProximity() * 100));
            NetworkTables::UpdateValue(networkTableKey + "/colorHue", std::to_string(getHue()));
            NetworkTables::UpdateValue(networkTableKey + "/colorSaturation", std::to_string(getSaturation() * 100));
            NetworkTables::UpdateValue(networkTableKey + "/colorBrightness", std::to_string(getBrightness() * 100));

            if (!sensor.is_installed())
                NetworkTables::UpdateValue(networkTableKey + "/faults", "Disconnected");
            else
                NetworkTables::UpdateValue(networkTableKey + "/faults", "");
        }

    private:
        static constexpr bool LOGGING_ENABLED = false;

        std::string name;
        pros::Optical sensor;
    };
}