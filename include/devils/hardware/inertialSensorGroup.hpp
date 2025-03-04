#pragma once

#include <string>
#include "structs/gyro.h"
#include "inertialSensor.hpp"
#include "pros/error.h"

namespace devils
{
    /**
     * Represents a set of Inertial Sensors fused together.
     */
    class InertialSensorGroup : public IGyro
    {
    public:
        /**
         * Creates a new IMU group.
         * @param name The name of the IMU group (for logging purposes)
         * @param ports The ports of the IMUs in the group (from 1 to 21)
         */
        InertialSensorGroup(std::string name, std::initializer_list<int8_t> ports)
            : name(name), sensors()
        {
            sensors.reserve(ports.size());
            for (int8_t port : ports)
                sensors.push_back(std::make_shared<InertialSensor>(getSensorName(port), port));
        }

        /**
         * Gets the average heading of all IMUs in the group.
         * @return The average heading of all IMUs in the group.
         */
        double getHeading() override
        {
            // Iterate through sensors and get average heading
            int sensorCount = 0;
            double heading = 0;
            for (auto sensor : sensors)
            {
                double sensorHeading = sensor->getHeading();

                // Skip sensors that fail to return heading
                if (sensorHeading == PROS_ERR_F)
                    continue;

                heading += sensorHeading;
                sensorCount++;
            }

            // Log if no sensors returned heading
            if (sensorCount == 0)
            {
                if (LOGGING_ENABLED)
                    Logger::warn(name + ": no sensors returned heading");
                return 0;
            }

            // Return the mean heading
            return heading / sensorCount;
        }

        /**
         * Sets the current heading of all IMUs in the group.
         * @param heading The heading to set the IMUs to in radians.
         */
        void setHeading(double heading) override
        {
            for (auto sensor : sensors)
                sensor->setHeading(heading);
        }

        /**
         * Gets the sensors in the inertial sensor group.
         * @return The sensors in the inertial sensor group.
         */
        std::vector<std::shared_ptr<InertialSensor>> &getSensors()
        {
            return sensors;
        }

        /**
         * Gets the name of each sensor in the inertial sensor group.
         */
        std::string getSensorName(int32_t port)
        {
            return name + "_" + std::to_string(port);
        }

    private:
        static constexpr bool LOGGING_ENABLED = false;

        const std::string name;
        std::vector<std::shared_ptr<InertialSensor>> sensors;
    };
}