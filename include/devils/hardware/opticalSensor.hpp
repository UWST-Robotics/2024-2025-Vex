#pragma once
#include "pros/imu.hpp"
#include "pros/optical.hpp"
#include "pros/error.h"
#include "../utils/logger.hpp"
#include "../geometry/units.hpp"
#include "../nt/objects/ntHardware.hpp"
#include <string>

namespace devils
{
    /**
     * Represents a V5 optical sensor unit.
     */
    class OpticalSensor : private NTHardware
    {
    public:
        /**
         * Creates a new Optical Sensor.
         * @param name The name of the Optical Sensor (for logging purposes)
         * @param port The port of the Optical Sensor (from 1 to 21)
         */
        OpticalSensor(const std::string name, const uint8_t port)
            : NTHardware(name, "OpticalSensor", port),
              sensor(port)
        {
            if (errno != 0)
                reportFault("Invalid port");
        }

        /**
         * Gets the current proximity of the Optical Sensor.
         * @return The current proximity of the Optical Sensor as an arbitrary value from 0 to 1 where 1 is the closest.
         */
        double getProximity()
        {
            std::int32_t proximity = sensor.get_proximity();
            if (proximity == PROS_ERR)
            {
                reportFault("Failed to retrieve proximity");
                return 0;
            }
            return proximity / 255.0;
        }

        /**
         * Gets the current hue of the Optical Sensor.
         * @return The current hue of the Optical Sensor as a value from 0 to 360.
         */
        double getHue()
        {
            std::int32_t hue = sensor.get_hue();
            if (hue == PROS_ERR)
            {
                reportFault("Failed to retrieve hue");
                return 0;
            }
            return hue;
        }

        /**
         * Gets the current saturation of the Optical Sensor.
         * @return The current saturation of the Optical Sensor as an percentage value from 0 to 1.
         */
        double getSaturation()
        {
            std::int32_t saturation = sensor.get_saturation();
            if (saturation == PROS_ERR)
            {
                reportFault("Failed to retrieve saturation");
                return 0;
            }
            return saturation;
        }

        /**
         * Gets the current brightness of the Optical Sensor.
         * @return The current brightness of the Optical Sensor as an percentage value from 0 to 1.
         */
        double getBrightness()
        {
            std::int32_t brightness = sensor.get_brightness();
            if (brightness == PROS_ERR)
            {
                reportFault("Failed to retrieve brightness");
                return 0;
            }
            return brightness;
        }

    protected:
        void serializeHardware(std::string &ntPrefix) override
        {
            NetworkTables::UpdateValue(ntPrefix + "/proximity", getProximity() * 100);
            NetworkTables::UpdateValue(ntPrefix + "/colorHue", getHue());
            NetworkTables::UpdateValue(ntPrefix + "/colorSaturation", getSaturation() * 100);
            NetworkTables::UpdateValue(ntPrefix + "/colorBrightness", getBrightness() * 100);
        }

        void checkHealth() override
        {
            if (!sensor.is_installed())
                reportFault("Disconnected");
            else
                clearFaults();
        }

    private:
        pros::Optical sensor;
    };
}